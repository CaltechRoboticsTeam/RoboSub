# CVAT labeling guide

Use this workflow to turn December's game objects and underwater recordings
into a shared, reviewed dataset. Students label **individual object instances**
with polygons or masks; the vision lead prepares jobs and releases datasets.
The actual class list and example images come after the game reveal.

## Student quick start

1. Get the team's CVAT URL, organization/project, assigned job link, and label
   examples from the vision lead. Use your own account and select the team's
   organization if one is configured. The OSU server in the reference guide
   belongs to their team.
2. Read the project's annotation instructions. Confirm which physical parts
   belong to each class, then open your assigned job. Work on that job only.
3. Label 10–20 frames and ask the reviewer to check them before continuing.
   This catches disagreements before they spread through hundreds of images.
4. On each frame, label every recognizable target instance, including small
   and partially hidden ones. Follow the boundary rules below.
5. Use **Save** regularly and wait for saving to finish before leaving.
   Finishing a polygon or mask does not by itself save the job to the server.
6. Check all frames, save, and hand the job to its reviewer. Use the team's
   assigned review transition; button names vary by CVAT version. Completing
   annotation is not the team's approval to include it in a dataset release.

If a class or boundary is unclear, record the frame and object ID in a CVAT
issue or the job handoff. Ask the reviewer; do not invent a new label or mark
an unresolved frame as a verified empty image.

## Drawing polygons and masks

For independent images, use **Shape** mode. Each physical object gets its own
instance, even when two objects of the same class touch.

- **Polygon:** select the polygon tool and class, then click around the visible
  boundary. Put vertices at corners and changes in curvature. Close the shape
  using the tool's finish control; use the editor's shortcut help for bindings.
  Avoid self-intersections and excessive vertices along straight edges.
- **Mask:** select the brush tool and class, paint the object's pixels, and
  erase background pixels. Use a smaller brush near edges. A mask can contain
  holes and disconnected regions belonging to the same object. Confirm
  **Save mask**, then use the job's **Save** button. See the official
  [brush instructions][brush] for the controls in your version.

Zoom in to place edges and zoom out to check for missed objects. Lower overlay
opacity when checking a boundary. A simple polygon cannot directly represent
a transparent hole; use a brush mask for objects with openings rather than
painting the opening as solid material.

### Boundary rules

Our default is **visible material only**, not the object's guessed full shape.
The lead must supply labeled examples for any class-specific exceptions.

| Situation | What to label |
| --- | --- |
| Partly hidden object | Put its visible regions in one instance mask. |
| Cropped object | Stop at the image border. |
| Hole or open frame | Leave background through the opening unmasked. |
| Two touching objects | Separate instances with separate boundaries. |
| Glare or shadow | Include object material; exclude spill and cast shadows. |
| Bubbles or reflections | Background unless the specification says otherwise. |
| Uncertain identity or boundary | Flag for review. |
| No target present | Leave empty after checking the entire frame. |

Do not merge instances into a crowd annotation. Do not duplicate one object
with both a polygon and a mask. If the target includes several physical parts,
follow the lead's examples for which parts belong to the same instance.

### Video jobs

Start with sampled images for the first labeling round. If the lead assigns
a video job using polygon **Track** mode, continue the existing object track
instead of creating a new one on every frame. Keep its first vertex and
clockwise/counterclockwise direction consistent between keyframes.

Interpolation is a starting point: inspect every frame that will be exported,
especially during turns, occlusions, and entry/exit. Add corrective keyframes.
Mark a track **Outside** when the object disappears and restore it when it
returns. The eye icon only hides the overlay. These track controls are covered
in the [OSU guide][osu]; do not assume brush masks interpolate like polygons.

## Vision lead: prepare the labeling round

### Collect useful images

Keep original recordings. Extract unmodified images from the same rectified
left RGB stream used by [our perception pipeline](vision.md), without debug
overlays. Preserve resolution and record the source session, timestamp,
camera/image mode, housing, and calibration version. Keep synchronized depth
and calibration data separately when collecting geometry evaluation data.
CVAT masks provide 2D labels, not calibration or measured 3D poses.

Collect different views, rotations, distances across the intended 0.5–2 m
range, backgrounds, lighting, and water conditions. Include multiple targets,
partial occlusion, cropped objects, and empty scenes with distracting objects.
Sample video sparsely to avoid paying for nearly identical frames; 1–2 fps is
a starting point, adjusted to motion and useful changes in appearance.

Assign whole recording sessions to **train**, **val**, or **test** before
sampling frames. A roughly 80/10/10 split is a starting point when enough
independent sessions exist. Adjacent frames, stereo partners, duplicates, and
augmented copies stay in the same split. Hold out separate collection sessions
for validation and testing; a random frame split can greatly overstate quality.

### Create the project and jobs

1. Choose a CVAT instance and give students access. Record its URL, version,
   storage location, and owner. For a team Linux server, use our
   [Docker setup](https://github.com/CaltechRoboticsTeam/RoboSub/tree/Zed-2i-Calib/deploy/cvat).
   Arrange hosting and accounts before the labeling round.
2. Create one project for the agreed label schema. Match class names exactly
   to `targets[].detector_class` in the vision configuration. Do not create
   different classes for each view or lighting condition.
3. Add a short specification with positive/negative examples, included parts,
   holes, occlusions, and ambiguous cases. Freeze names and class order for the
   round; version any later changes and update affected annotations.
4. Create tasks within that project, one per recording session and split.
   Set each task's subset to `train`, `val`, or `test`. Use globally unique
   image names such as `session03_left_000142.png`.
5. Use job segments of about 50–100 sampled images initially. Assign one
   student and a reviewer to each job. Adjust size after timing the pilot;
   complex masks take longer than boxes. Review the pilot before scaling up.

If a solver requires landmarks, define a separate skeleton/keypoint policy
before labeling. Planar targets need ordered physical TL, TR, BR, BL corners
in the object's local frame. Polygon vertices do not supply these landmarks.

Share this handoff for each batch:

```text
CVAT URL / organization / project:
Label specification version and examples:
Job link / source session / split:
Annotator / reviewer:
Known ambiguities and frame IDs:
```

## Review before release

Review every job, including empty frames. Check class names, missed instances,
boundaries, holes, duplicate objects, and accidental merges. In video jobs,
check interpolated frames as well as keyframes. Return concrete issues with
frame IDs; the student fixes them and the reviewer checks again.

Track annotation, review, and acceptance separately using CVAT job stages
where available, or a shared job list. **Save/Finish** controls are described
in the [CVAT editor guide][editor]. A completed annotation job enters review;
only reviewer-approved jobs enter a release. Export from a reviewed snapshot
or pause edits during export so unreviewed changes do not slip in.

After a useful first model exists, the lead may use its predictions to prepare
later training jobs. Students must correct every proposed mask/box and add
missed objects. Record the model version and keep held-out evaluation sessions
fixed. Use manual tools or the team's own approved models; do not use SAM.

## Export and train

Keep the reviewed masks as the master dataset. For each numbered release:

1. Save a native CVAT backup and export **COCO instance annotations with
   images**. CVAT's [COCO format][coco] supports polygons and masks. Preserve
   holes as masks/RLE, rather than forcing every object into a single polygon.
   Leave the `is_crowd` label attribute unset and keep individual instances.
   Verify the exported masks with holes use RLE.
2. Archive the class map, split/session list, label specification, CVAT
   version, review record, and source filenames with the release. Keep large
   images and archives outside Git; `vision_data/` is ignored by this repo.
3. Re-import a small exported sample into a scratch task. Compare instances,
   holes, disconnected regions, and empty frames to the originals. Inspect
   class IDs and counts in every split before handing it to training.

### Choose the right training labels

The current `robosub_vision` detector accepts YOLO **detection** or **pose**
models. It does not consume segmentation model outputs. Keep masks now so
they remain available for future segmentation work; derive boxes for the
existing detector. Mask training/deployment requires a separate pipeline
change. See [underwater vision](vision.md#add-targets-and-train).

| Intended use | Export/preparation |
| --- | --- |
| Detection model | Derive boxes from reviewed COCO instances. |
| Pose model | Label skeletons; export Ultralytics YOLO Pose. |
| Future segmentation model | Export YOLO Segmentation; check mask losses. |

CVAT's [Ultralytics formats][yolo] export Detection from **boxes**, Segmentation
from **polygons/masks**, and Pose from **skeletons**. Selecting Detection on a
polygon-only project is not a mask-to-box conversion. Use standard detection
labels without an appended tracking ID. YOLO polygon labels can lose holes
or join disconnected regions; retain the original masks as the master.

For a box dataset, make a **staging copy** of the COCO JSON files under
`vision_data/release_v1/coco_boxes/annotations`. Check/remap category IDs and
each annotation's `category_id` to `1..N` in the frozen class order.

**Check `iscrowd` before conversion.** CVAT's [Datumaro exporter][datumaro] can
write `iscrowd: 1` for a single brush mask because it uses RLE. Ultralytics
skips those annotations. In the staging copy only, set `iscrowd: 0` for masks
reviewed as individual objects. Remove their `segmentation` fields to make this
a box-only copy. Resolve any actual merged crowds manually. Preserve the
original mask export unchanged. Then use Ultralytics' [converter][converter]:

```python
from ultralytics.data.converter import convert_coco

convert_coco(
    labels_dir="vision_data/release_v1/coco_boxes/annotations",
    save_dir="vision_data/release_v1/yolo_detect",
    use_segments=False,
    cls91to80=False,
)
```

Use a fresh output directory. This converts COCO boxes to YOLO labels; it does
not copy the images. Place the original images under matching
`images/{train,val,test}` folders alongside `labels/{train,val,test}`. Check
paths against each JSON's `file_name`; retain all verified negative images and
give them empty label files. Compare source and converted instance counts to
catch dropped targets. YOLO class IDs are COCO IDs minus one.

Create `data.yaml` with an absolute dataset `path`, `train: images/train`,
`val: images/val`, `test: images/test`, and `names` mapping `0..N-1` to the
frozen class names. Render sample label overlays before starting a long run.
Train with the existing command and a local detection checkpoint:

```bash
python3 tools/train_vision.py train --model /path/to/yolo11n.pt \
  --data vision_data/release_v1/yolo_detect/data.yaml --epochs 100
```

Record the dataset release and model checkpoint with each training run.
Inspect validation errors to choose the next collection/labeling batch;
reserve the test sessions for final evaluation.

## References

The [OSU UWRT CVAT usage guide][osu] informed the student workflow. Server
addresses, team contacts, and game-specific labels in that guide are theirs.
Use the official links above for current controls and export behavior.

[osu]: https://osu-uwrt-1.gitbook.io/computer-vision-guide/cvat-usage-and-labeling-guide/cvat-usage-guide
[brush]: https://docs.cvat.ai/docs/annotation/manual-annotation/shapes/annotation-with-brush-tool/
[editor]: https://docs.cvat.ai/docs/annotation/annotation-editor/navbar/
[coco]: https://docs.cvat.ai/docs/dataset_management/formats/format-coco/
[yolo]: https://docs.cvat.ai/docs/dataset_management/formats/format-yolo-ultralytics/
[converter]: https://docs.ultralytics.com/reference/data/converter/
[datumaro]: https://github.com/open-edge-platform/datumaro/blob/develop/src/datumaro/plugins/data_formats/coco/exporter.py
