## Demo

[![Watch the demo](media/demo.gif)](https://github.com/gygigies/Assignment4/blob/main/demo.mp4)

---

## Controls

- **Mouse**: rotate camera (player always faces camera yaw)  
- **WASD**: move relative to camera  
- **Shift**: sprint  
- **Space**: jump  
- **LMB**: shoot  
- **U / J**: raise/lower map `MAP_Y_OFFSET` (colliders rebuild automatically)  
- **PgUp/PgDn**: adjust `PLAYER_FOOT_BIAS`, **Home/End**: `ENEMY_FOOT_BIAS`  
- **F1**: toggle wireframe  
- **ESC**: quit

---

## What’s inside

- `main.cpp` — game loop, camera, movement, jump physics, shooting  
- `shaders/1.model_loading.vs`, `shaders/1.model_loading.fs` — standard LearnOpenGL PBR-ish textured model shader  
- `resources/` — **only project-owned files** (placeholders and your own textures).  

Core techniques:
- **Map collision build**: for each triangle in the OBJ scene:  
  - Triangles with `normal.y >= FLOOR_MIN_NY` become **floors**.  
  - Triangles with `abs(normal.y) <= WALL_MAX_NY` contribute to **wall AABBs** (XZ).  
- **Floor sampling**: Möller–Trumbore raycast straight down to find floor Y at a given XZ.  
- **Movement**: move on XZ plane; AABB push-out for walls (multi-pass unstick).  
- **Step-up / Step-down**: if target floor is within thresholds, snap up/down smoothly.

---

## Build

This project uses the LearnOpenGL-style helper classes (`Shader`, `Camera`, `Model`, `FileSystem`).  
Any modern C++17 compiler works.

## Credits (3rd-party assets)

"Cuphead" (https://skfb.ly/6uD78) by Boros is licensed under Creative Commons Attribution (http://creativecommons.org/licenses/by/4.0/).

"WXL7HFRJ6939KC6MFQ0H23PNX_obj" (https://skfb.ly/oEUZO) by rogeriofsalves1 is licensed under Creative Commons Attribution (http://creativecommons.org/licenses/by/4.0/).

"Minecraft 3D Map Desert Village (obj + texture)" (https://skfb.ly/oQVKL) by HuyAC is licensed under Creative Commons Attribution (http://creativecommons.org/licenses/by/4.0/).

"Peeled Banana 22" (https://skfb.ly/6RpV7) by deep3dstudio is licensed under Creative Commons Attribution-NonCommercial (http://creativecommons.org/licenses/by-nc/4.0/).
