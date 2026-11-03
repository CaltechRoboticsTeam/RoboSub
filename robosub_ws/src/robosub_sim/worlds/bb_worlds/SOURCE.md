Assets copied from [BumblebeeAS/bb_worlds](https://github.com/BumblebeeAS/bb_worlds)
at commit `94e7e8617fc9fedb022c4c11918b8021b2858751`, under the included MIT
license. Git LFS contents were downloaded and checked against their pointer
SHA-256 hashes and sizes.

Includes `models/woollett_pool`, the RoboSub 2026 gate and its base meshes,
and the two gate task textures. The pool GLB embeds its floor texture.
The upstream pool model is 50 x 22.86 x 2.1336 m; these are model dimensions,
not a newly verified survey of the competition venue.

`simulation_world.py` resolves local asset paths, places the gate at
`(6, 0, -2.1336)` facing the forward test corridor, and gives its collision
mesh the same Y-up-to-Z-up rotation as its visual. The sub starts at 1 m
depth. Vehicle physics, sensors and lighting come from our pool template;
this is a small adaptation, not Bumblebee's complete competition world.
