#!/usr/bin/env python3
"""Generate vision datasets with Isaac Sim on the workstation."""

import argparse

import _vision  # noqa: F401


def run(args, application):
    import cv2
    import numpy as np
    import omni.replicator.core as rep
    import omni.usd
    import trimesh
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade
    from robosub_vision.synthetic import (
        DatasetWriter,
        load_scene,
        random_pose,
        underwater_appearance,
    )

    scene = load_scene(args.scene)
    writer = DatasetWriter(
        args.output, scene, args.frames, args.seed, "isaac_replicator_pinhole"
    )
    rng = np.random.default_rng(args.seed)
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    # USD: -Z forward, +Y up. Export: optical +Z forward, +Y down.
    optical_to_world = np.diag([1.0, -1.0, -1.0])

    def material(path, color):
        mat = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        diffuse = shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
        diffuse.Set(Gf.Vec3f(*map(float, color)))
        mat.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface"
        )
        return mat, diffuse

    def make_mesh(path, geometry, label=None):
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(
            [Gf.Vec3f(*map(float, p)) for p in geometry.vertices]
        )
        mesh.CreateFaceVertexCountsAttr([3] * len(geometry.faces))
        mesh.CreateFaceVertexIndicesAttr(geometry.faces.ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mat, diffuse = material(path + "Material", rng.uniform(0.1, 0.9, 3))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        if label:
            rep.modify.semantics([("class", label)], input_prims=[path])
        transform = UsdGeom.Xformable(mesh).AddTransformOp()
        return mesh, transform, diffuse

    objects = [
        make_mesh(f"/World/Target{i}", target["geometry"], target["name"])
        for i, target in enumerate(scene["targets"])
    ]
    backdrop = make_mesh(
        "/World/Background", trimesh.creation.box([10, 10, 0.1])
    )
    backdrop[1].Set(Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, -4)))
    occluder = make_mesh(
        "/World/Occluder", trimesh.creation.box([0.08, 0.2, 0.05])
    )
    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateProjectionAttr("perspective")
    camera.CreateFocalLengthAttr(24.0)
    width, height = writer.size
    camera.CreateHorizontalApertureAttr(24 * width / writer.k[0, 0])
    camera.CreateVerticalApertureAttr(24 * height / writer.k[1, 1])
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10))
    dome = UsdLux.DomeLight.Define(stage, "/World/Ambient")
    dome.CreateIntensityAttr(200.0)
    light = UsdLux.SphereLight.Define(stage, "/World/Light")
    light.CreateRadiusAttr(0.2)
    light_pose = UsdGeom.Xformable(light).AddTranslateOp()
    product = rep.create.render_product(str(camera.GetPath()), writer.size)
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    depth_annotator = rep.AnnotatorRegistry.get_annotator(
        "distance_to_image_plane"
    )
    boxes = rep.AnnotatorRegistry.get_annotator(
        "bounding_box_2d_tight", init_params={"semanticTypes": ["class"]}
    )
    for annotator in (rgb, depth_annotator, boxes):
        annotator.attach(product)
    for index in range(args.frames):
        class_id = index % len(objects)
        r, t = random_pose(rng, scene.get("distance", [0.5, 2.0]))
        for i, (mesh, transform, diffuse) in enumerate(objects):
            UsdGeom.Imageable(mesh).GetVisibilityAttr().Set(
                "inherited" if i == class_id else "invisible"
            )
            if i == class_id:
                pose = np.eye(4)
                pose[:3, :3] = optical_to_world @ r
                pose[:3, 3] = optical_to_world @ t
                transform.Set(Gf.Matrix4d(*pose.T.ravel().tolist()))
                diffuse.Set(Gf.Vec3f(*map(float, rng.uniform(0.1, 0.9, 3))))
        occlude = rng.random() < 0.35
        UsdGeom.Imageable(occluder[0]).GetVisibilityAttr().Set(
            "inherited" if occlude else "invisible"
        )
        front = optical_to_world @ (t + [rng.uniform(-0.1, 0.1), 0, -0.2])
        occluder[1].Set(Gf.Matrix4d().SetTranslate(Gf.Vec3d(*front)))
        backdrop[2].Set(Gf.Vec3f(*map(float, rng.uniform(0.05, 0.6, 3))))
        light.GetIntensityAttr().Set(float(rng.uniform(300, 2500)))
        light_pose.Set(Gf.Vec3d(*rng.uniform([-2, -2, 0], [2, 2, 1])))
        rep.orchestrator.step(rt_subframes=4)
        rgba = np.asarray(rgb.get_data())
        depth = np.asarray(depth_annotator.get_data()).reshape(height, width)
        image = cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
        image = underwater_appearance(image, depth, rng)
        annotation = boxes.get_data()
        box = None
        for row in annotation["data"]:
            semantic_id = row["semanticId"]
            labels = annotation["info"]["idToLabels"]
            label = labels.get(
                str(semantic_id), labels.get(int(semantic_id), {})
            )
            if label.get("class") == scene["targets"][class_id]["name"]:
                box = [
                    int(row[key])
                    for key in ("x_min", "y_min", "x_max", "y_max")
                ]
                if box[2] <= box[0] or box[3] <= box[1]:
                    box = None
                break
        writer.write(index, class_id, image, depth, box, r, t)
    rep.orchestrator.stop()
    application.update()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--frames", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=135)
    args = parser.parse_args()
    if args.frames < 10:
        parser.error("Use >=10 frames")
    from isaacsim import SimulationApp

    application = SimulationApp({"headless": True})
    try:
        run(args, application)
    finally:
        application.close()


if __name__ == "__main__":
    main()
