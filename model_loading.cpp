// ===================== Center TPS — Map OBJ + Jump + Collisions + Step-up (Height-Aware Walls) =====================
// Controls:
//   Mouse  : rotate camera (player faces camera yaw)
//   WASD   : move relative to camera
//   Shift  : sprint
//   Space  : jump
//   LMB    : shoot
//   U / J  : raise / lower map (rebuilds collider)
//   PgUp/Dn: PLAYER_FOOT_BIAS,  Home/End: ENEMY_FOOT_BIAS
//   F1     : toggle wireframe
//   ESC    : quit

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <learnopengl/filesystem.h>
#include <learnopengl/shader_m.h>
#include <learnopengl/camera.h>
#include <learnopengl/model.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <limits>

// ---------- Map path ----------
static const char* MAP_MODEL_RELATIVE_PATH =
"resources/objects/desert/desert_vill.obj";

// ---------- Window ----------
const unsigned int SCR_WIDTH = 1280;
const unsigned int SCR_HEIGHT = 720;

// ---------- Time ----------
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// ---------- Camera (centered TPS) ----------
Camera camera(glm::vec3(0.0f, 2.3f, 5.0f));
bool   firstMouse = true;
float  lastX = SCR_WIDTH * 0.5f;
float  lastY = SCR_HEIGHT * 0.5f;

float gCamYawDeg = 0.0f;
float gCamPitchDeg = -15.0f;

float gPlayerYawDeg = 0.0f;
float gCamDistance = 5.2f;
float gCamHeight = 2.4f;
float gCamSmooth = 0.18f;

// ---------- Movement ----------
float gWalkSpeed = 10.5f;
float gSprintMul = 1.6f;

// ---------- Vertical physics (jump) ----------
float gVelY = 0.0f;
bool  gGrounded = false;
const float GRAVITY = 25.0f;
const float JUMP_FORCE = 9.5f;

// ---------- Step-up / step-down helpers ----------
const float STEP_MAX = 1.25f; // max height we can step up
const float STEP_SNAP_EPS = 0.03f;
const float STEP_DOWN_MAX = 1.35f;

// Unstick from corners
const float SKIN = 0.04f;
const int   UNSTICK_ITER = 4;

// ---------- Shooting ----------
bool  gShootHeld = false;
float gShootCooldown = 0.0f;

// ---------- Debug ----------
bool gWire = false;

// ---------- Bullets ----------
struct Bullet {
    glm::vec3 pos{ 0 };
    glm::vec3 vel{ 0 };
    float life{ 4.0f };
    float radius{ 0.2f };
    bool  active{ true };
};
std::vector<Bullet> gBullets;

// ---------- 2D AABB (XZ) ----------
struct AABB2D { glm::vec2 center, halfExt; };
static inline bool IntersectsXZ(const AABB2D& a, const AABB2D& b) {
    if (std::abs(a.center.x - b.center.x) > (a.halfExt.x + b.halfExt.x)) return false;
    if (std::abs(a.center.y - b.center.y) > (a.halfExt.y + b.halfExt.y)) return false;
    return true;
}
static inline void ResolveStaticXZ(const AABB2D& statBox, AABB2D& dynBox, glm::vec3& posXZ) {
    float dx = dynBox.center.x - statBox.center.x;
    float dz = dynBox.center.y - statBox.center.y;
    float px = (dynBox.halfExt.x + statBox.halfExt.x) - std::abs(dx);
    float pz = (dynBox.halfExt.y + statBox.halfExt.y) - std::abs(dz);
    if (px < 0 || pz < 0) return;
    if (px < pz) { float sx = (dx < 0 ? -1.f : 1.f); posXZ.x += sx * px; dynBox.center.x += sx * px; }
    else { float sz = (dz < 0 ? -1.f : 1.f); posXZ.z += sz * pz; dynBox.center.y += sz * pz; }
}

// ---------- Map placement ----------
float MAP_Y_OFFSET = -20.0f;
float MAP_SCALE = 1.0f;
float MAP_YAW_DEG = 0.0f;

float PLAYER_FOOT_BIAS = 1.15f;
float ENEMY_FOOT_BIAS = 1.15f;

// ---------- Map collision data ----------
struct Tri { glm::vec3 a, b, c; glm::vec3 n; };

// Height-aware wall box
struct WallBox {
    AABB2D boxXZ;
    float  minY, maxY;
};

std::vector<Tri>     gFloorTris;
std::vector<WallBox> gWalls;

const float FLOOR_MIN_NY = 0.55f; // treat tri as floor if normal.y >= this
const float WALL_MAX_NY = 0.25f; // treat tri as wall if |normal.y| <= this

glm::mat4 MapTransform() {
    glm::mat4 M(1.0f);
    M = glm::translate(M, glm::vec3(0.f, MAP_Y_OFFSET, 0.f));
    M = glm::rotate(M, glm::radians(MAP_YAW_DEG), glm::vec3(0, 1, 0));
    M = glm::scale(M, glm::vec3(MAP_SCALE));
    return M;
}

void BuildMapCollision(const Model& mapModel) {
    gFloorTris.clear();
    gWalls.clear();

    glm::mat4 T = MapTransform();
    for (const auto& mesh : mapModel.meshes) {
        const auto& V = mesh.vertices;
        const auto& I = mesh.indices;
        if (I.empty() || V.empty()) continue;

        for (size_t i = 0; i + 2 < I.size(); i += 3) {
            glm::vec3 a = glm::vec3(T * glm::vec4(V[I[i + 0]].Position, 1.0));
            glm::vec3 b = glm::vec3(T * glm::vec4(V[I[i + 1]].Position, 1.0));
            glm::vec3 c = glm::vec3(T * glm::vec4(V[I[i + 2]].Position, 1.0));
            glm::vec3 n = glm::normalize(glm::cross(b - a, c - a));
            float ny = std::abs(n.y);

            if (n.y >= FLOOR_MIN_NY) {
                gFloorTris.push_back({ a,b,c,n });
            }
            else if (ny <= WALL_MAX_NY) {
                // Build a height-aware wall box
                float minx = std::min({ a.x,b.x,c.x });
                float maxx = std::max({ a.x,b.x,c.x });
                float minz = std::min({ a.z,b.z,c.z });
                float maxz = std::max({ a.z,b.z,c.z });
                float miny = std::min({ a.y,b.y,c.y });
                float maxy = std::max({ a.y,b.y,c.y });
                const float m = 0.06f;
                minx -= m; maxx += m; minz -= m; maxz += m;

                WallBox w;
                w.boxXZ = { {(minx + maxx) * 0.5f,(minz + maxz) * 0.5f},
                            {(maxx - minx) * 0.5f,(maxz - minz) * 0.5f} };
                w.minY = miny;
                w.maxY = maxy;
                gWalls.push_back(w);
            }
        }
    }
    std::cout << "[MapCollider] floors=" << gFloorTris.size()
        << " walls=" << gWalls.size() << "\n";
}

// Möller–Trumbore
bool RaycastTri(const glm::vec3& ro, const glm::vec3& rd, const Tri& tri, float& tHit) {
    const float EPS = 1e-6f;
    glm::vec3 v0v1 = tri.b - tri.a;
    glm::vec3 v0v2 = tri.c - tri.a;
    glm::vec3 pvec = glm::cross(rd, v0v2);
    float det = glm::dot(v0v1, pvec);
    if (fabs(det) < EPS) return false;
    float invDet = 1.0f / det;

    glm::vec3 tvec = ro - tri.a;
    float u = glm::dot(tvec, pvec) * invDet; if (u < 0.0f || u > 1.0f) return false;
    glm::vec3 qvec = glm::cross(tvec, v0v1);
    float v = glm::dot(rd, qvec) * invDet;   if (v < 0.0f || (u + v) > 1.0f) return false;

    float t = glm::dot(v0v2, qvec) * invDet; if (t <= 0.0f) return false;
    tHit = t; return true;
}

float SampleFloorY(const glm::vec3& worldPosXZ) {
    glm::vec3 ro(worldPosXZ.x, 1000.0f, worldPosXZ.z);
    glm::vec3 rd(0, -1, 0);

    float bestT = std::numeric_limits<float>::infinity();
    bool  hit = false;

    for (const Tri& tri : gFloorTris) {
        float t;
        if (RaycastTri(ro, rd, tri, t)) {
            if (t > 0.0f && t < bestT) { bestT = t; hit = true; }
        }
    }
    if (hit) return ro.y + bestT * rd.y;
    return MAP_Y_OFFSET;
}

// Height-aware wall check
inline bool IntersectsWallAtHeight(const WallBox& w, const AABB2D& ply, float footY) {
    const float Y_PAD_DOWN = 0.6f;  // allow a bit of overlap below feet
    const float Y_PAD_UP = 1.8f;  // wall height that can block (roughly up to chest/head)
    if (footY < (w.minY - Y_PAD_DOWN) || footY >(w.maxY + Y_PAD_UP)) return false;
    return IntersectsXZ(ply, w.boxXZ);
}

void ResolveStaticWall(const WallBox& w, AABB2D& dynBox, glm::vec3& posXZ) {
    ResolveStaticXZ(w.boxXZ, dynBox, posXZ);
}

// ---------- Callbacks ----------
void framebuffer_size_callback(GLFWwindow*, int w, int h) { glViewport(0, 0, w, h); }

void mouse_callback(GLFWwindow*, double xposIn, double yposIn) {
    float xpos = (float)xposIn, ypos = (float)yposIn;
    if (firstMouse) { lastX = xpos; lastY = ypos; firstMouse = false; }
    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;
    lastX = xpos; lastY = ypos;

    const float sens = 0.12f;
    gCamYawDeg += xoffset * sens;
    gCamPitchDeg += yoffset * sens;
    gCamPitchDeg = glm::clamp(gCamPitchDeg, -45.0f, 10.0f);

    gPlayerYawDeg = gCamYawDeg;
}
void scroll_callback(GLFWwindow*, double, double y) { camera.ProcessMouseScroll((float)y); }
void mouse_button_callback(GLFWwindow*, int button, int action, int) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) gShootHeld = (action == GLFW_PRESS);
}
bool keyDown(GLFWwindow* w, int k) { return glfwGetKey(w, k) == GLFW_PRESS; }

void updateWindowTitleWithBias(GLFWwindow* window) {
    std::string title = "Center TPS (Map OBJ) | PlayerBias=" + std::to_string(PLAYER_FOOT_BIAS)
        + " EnemyBias=" + std::to_string(ENEMY_FOOT_BIAS)
        + " MapY=" + std::to_string(MAP_Y_OFFSET);
    glfwSetWindowTitle(window, title.c_str());
}

// Move with step-up; fallback to AABB push resolve
bool TryMoveWithStepUp(glm::vec3& posXZ, const glm::vec3& moveXZ, float currentFootY,
    AABB2D& playerBox, float& outNewFootY)
{
    glm::vec3 candidate = posXZ + moveXZ;
    AABB2D candBox = playerBox; candBox.center = { candidate.x, candidate.z };

    bool collide = false;
    for (const auto& w : gWalls) { if (IntersectsWallAtHeight(w, candBox, currentFootY)) { collide = true; break; } }

    if (!collide) { // free move
        posXZ = candidate; playerBox.center = candBox.center;
        outNewFootY = currentFootY;
        return true;
    }

    // allow step-up if new floor is slightly higher
    float newFloorY = SampleFloorY(candidate);
    float diff = newFloorY - currentFootY;
    if (diff > -STEP_SNAP_EPS && diff <= STEP_MAX) {
        // also make sure at new height we aren't inside walls
        bool collideNew = false;
        for (const auto& w : gWalls) { if (IntersectsWallAtHeight(w, candBox, newFloorY)) { collideNew = true; break; } }
        if (!collideNew) {
            posXZ = candidate; playerBox.center = candBox.center;
            outNewFootY = newFloorY;
            return true;
        }
    }
    return false;
}

void processInput(GLFWwindow* w, glm::vec3& playerPosXZ, AABB2D& playerBox,
    float& playerFootY, bool& mapDirty)
{
    if (keyDown(w, GLFW_KEY_ESCAPE)) glfwSetWindowShouldClose(w, true);

    static bool f1Prev = false; bool f1Now = keyDown(w, GLFW_KEY_F1);
    if (f1Now && !f1Prev) { gWire = !gWire; glPolygonMode(GL_FRONT_AND_BACK, gWire ? GL_LINE : GL_FILL); }
    f1Prev = f1Now;

    glm::vec3 forward = glm::normalize(glm::vec3(
        std::sin(glm::radians(gCamYawDeg)), 0.0f, -std::cos(glm::radians(gCamYawDeg))
    ));
    glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));

    glm::vec3 dir(0.0f);
    if (keyDown(w, GLFW_KEY_W)) dir += forward;
    if (keyDown(w, GLFW_KEY_S)) dir -= forward;
    if (keyDown(w, GLFW_KEY_A)) dir -= right;
    if (keyDown(w, GLFW_KEY_D)) dir += right;

    float speed = gWalkSpeed * (keyDown(w, GLFW_KEY_LEFT_SHIFT) ? gSprintMul : 1.0f);
    if (glm::length(dir) > 0.001f) {
        dir = glm::normalize(dir);
        glm::vec3 moveXZ = dir * speed * deltaTime;

        float footYCandidate = playerFootY;
        if (!TryMoveWithStepUp(playerPosXZ, moveXZ, playerFootY, playerBox, footYCandidate)) {
            // multi-pass push-out to reduce corner sticking
            AABB2D tmp = playerBox; tmp.center = { playerPosXZ.x, playerPosXZ.z };
            for (int it = 0; it < UNSTICK_ITER; ++it) {
                bool any = false;
                for (const auto& w : gWalls) {
                    if (IntersectsWallAtHeight(w, tmp, playerFootY)) { ResolveStaticWall(w, tmp, playerPosXZ); any = true; }
                }
                if (!any) break;
            }
            playerBox.center = tmp.center;
        }
        else {
            playerFootY = footYCandidate;
        }
    }

    // jump
    if (keyDown(w, GLFW_KEY_SPACE) && gGrounded) { gVelY = JUMP_FORCE; gGrounded = false; }

    // live tuning
    static bool prevPgUp = false, prevPgDn = false, prevHome = false, prevEnd = false;
    bool pgUp = keyDown(w, GLFW_KEY_PAGE_UP);
    bool pgDn = keyDown(w, GLFW_KEY_PAGE_DOWN);
    bool homeK = keyDown(w, GLFW_KEY_HOME);
    bool endK = keyDown(w, GLFW_KEY_END);

    if (pgUp && !prevPgUp) { PLAYER_FOOT_BIAS += 0.05f; updateWindowTitleWithBias(w); }
    if (pgDn && !prevPgDn) { PLAYER_FOOT_BIAS -= 0.05f; updateWindowTitleWithBias(w); }
    if (homeK && !prevHome) { ENEMY_FOOT_BIAS += 0.05f; updateWindowTitleWithBias(w); }
    if (endK && !prevEnd) { ENEMY_FOOT_BIAS -= 0.05f; updateWindowTitleWithBias(w); }

    prevPgUp = pgUp; prevPgDn = pgDn; prevHome = homeK; prevEnd = endK;

    static bool prevU = false, prevJ = false;
    bool U = keyDown(w, GLFW_KEY_U);
    bool J = keyDown(w, GLFW_KEY_J);
    if (U && !prevU) { MAP_Y_OFFSET += 0.10f; updateWindowTitleWithBias(w); mapDirty = true; }
    if (J && !prevJ) { MAP_Y_OFFSET -= 0.10f; updateWindowTitleWithBias(w); mapDirty = true; }
    prevU = U; prevJ = J;
}

// ---------- Follow camera ----------
void updateFollowCamera(const glm::vec3& playerPosAbs) {
    float yaw = glm::radians(gCamYawDeg);
    glm::vec3 forward = glm::normalize(glm::vec3(std::sin(yaw), 0, -std::cos(yaw)));
    glm::vec3 up(0, 1, 0);

    glm::vec3 desired = playerPosAbs - forward * gCamDistance + up * gCamHeight;
    camera.Position = glm::mix(camera.Position, desired, gCamSmooth);

    glm::vec3 lookTarget = playerPosAbs + glm::vec3(0, 1.0f, 0);
    camera.Front = glm::normalize(lookTarget - camera.Position);
    camera.Right = glm::normalize(glm::cross(camera.Front, up));
    camera.Up = glm::normalize(glm::cross(camera.Right, camera.Front));
}

// Try to nudge spawn out of walls if initial position is blocked
void NudgeSpawn(glm::vec3& posXZ, AABB2D& box, float& footY) {
    auto blocked = [&](const glm::vec3& p)->bool {
        AABB2D b = box; b.center = { p.x, p.z };
        for (const auto& w : gWalls) if (IntersectsWallAtHeight(w, b, footY)) return true;
        return false;
        };
    if (!blocked(posXZ)) return;

    const float STEP = 0.5f;
    for (int r = 1; r <= 40; ++r) {
        for (int dx = -r; dx <= r; ++dx) {
            for (int dz = -r; dz <= r; ++dz) {
                glm::vec3 c = posXZ + glm::vec3(dx * STEP, 0, dz * STEP);
                float y = SampleFloorY(c);
                float f = y;
                if (!blocked(c)) {
                    posXZ = c; box.center = { c.x, c.z };
                    footY = f;
                    std::cout << "[Spawn] nudged to (" << c.x << "," << c.z << ")\n";
                    return;
                }
            }
        }
    }
}

int main() {
    // Window
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Center TPS (Map OBJ)", nullptr, nullptr);
    if (!window) { std::cout << "Failed to create window\n"; return -1; }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) { std::cout << "Failed to init GLAD\n"; return -1; }
    stbi_set_flip_vertically_on_load(false);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE); glCullFace(GL_BACK);

    Shader shader("1.model_loading.vs", "1.model_loading.fs");

    Model playerModel(FileSystem::getPath("resources/objects/un/un.obj"));
    Model enemyModel(FileSystem::getPath("resources/objects/cuphead/cuphead_rig.obj"));
    Model itemModel(FileSystem::getPath("resources/objects/backpack/backpack.obj"));
    Model ballModel(FileSystem::getPath("resources/objects/banana/banana.obj")); // used as bullet mesh
    Model mapModel(FileSystem::getPath(MAP_MODEL_RELATIVE_PATH));

    // Build colliders from map geometry
    BuildMapCollision(mapModel);

    // State
    float playerScale = 1.0f;
    float enemyScale = 0.45f;

    glm::vec3 playerPosXZ(0, 0, 0);
    glm::vec3 enemyPosXZ(-6.f, 0.f, 2.f);
    glm::vec3 itemPosXZ(2.f, 0.f, -4.f);

    AABB2D playerBox{ {playerPosXZ.x,playerPosXZ.z},{0.40f,0.40f} };
    AABB2D enemyBox{ {enemyPosXZ.x, enemyPosXZ.z },{0.45f,0.45f} };
    AABB2D itemBox{ {itemPosXZ.x,  itemPosXZ.z  },{0.60f,0.60f} };

    glm::vec3 playerAbs(0), enemyAbs(0), itemAbs(0);

    float playerFootY = SampleFloorY(playerPosXZ);

    // If spawn overlaps walls at this height, nudge to a nearby free spot
    NudgeSpawn(playerPosXZ, playerBox, playerFootY);

    playerAbs = { playerPosXZ.x, playerFootY + PLAYER_FOOT_BIAS, playerPosXZ.z };

    bool  mapDirty = false;
    int   enemyHP = 7;
    float enemyAmp = 6.f, enemySpeed = 1.2f, enemyDir = 1.f;
    bool  itemCollected = false;

    auto drawAbs = [&](Model& m, const glm::vec3& pAbs, float yawDeg, float s) {
        glm::mat4 M(1.0f);
        M = glm::translate(M, pAbs);
        M = glm::rotate(M, glm::radians(yawDeg), glm::vec3(0, 1, 0));
        M = glm::scale(M, glm::vec3(s));
        shader.setMat4("model", M);
        m.Draw(shader);
        };

    while (!glfwWindowShouldClose(window)) {
        float t = (float)glfwGetTime();
        deltaTime = t - lastFrame; lastFrame = t;

        processInput(window, playerPosXZ, playerBox, playerFootY, mapDirty);
        if (mapDirty) { BuildMapCollision(mapModel); mapDirty = false; }

        // Anchor enemy & item to floor
        {
            float ey = SampleFloorY(enemyPosXZ);
            enemyAbs = { enemyPosXZ.x, ey + ENEMY_FOOT_BIAS, enemyPosXZ.z };
            float iy = SampleFloorY(itemPosXZ);
            itemAbs = { itemPosXZ.x,  iy + 0.05f,            itemPosXZ.z };
        }

        // Vertical physics
        gVelY -= GRAVITY * deltaTime;
        float proposedY = playerAbs.y + gVelY * deltaTime;

        float floorY = SampleFloorY(playerPosXZ);
        float minY = floorY + PLAYER_FOOT_BIAS;

        if (proposedY <= minY) {
            playerAbs.y = minY;
            gVelY = 0.0f;
            gGrounded = true;
            playerFootY = floorY;
        }
        else {
            playerAbs.y = proposedY;
            gGrounded = false;
        }
        playerAbs.x = playerPosXZ.x;
        playerAbs.z = playerPosXZ.z;

        // Unstick pass (height-aware)
        {
            AABB2D b = playerBox;
            b.halfExt.x = std::max(0.01f, b.halfExt.x - SKIN);
            b.halfExt.y = std::max(0.01f, b.halfExt.y - SKIN);

            for (int it = 0; it < UNSTICK_ITER; ++it) {
                bool any = false;
                for (const auto& w : gWalls) {
                    if (IntersectsWallAtHeight(w, b, playerFootY)) { ResolveStaticWall(w, b, playerPosXZ); any = true; }
                }
                if (!any) break;
            }
            playerBox.center = b.center;
            playerAbs.x = playerPosXZ.x;
            playerAbs.z = playerPosXZ.z;
        }

        // Step-down snap (smooth descent)
        {
            float newFloor = SampleFloorY(playerPosXZ);
            float drop = playerFootY - newFloor;
            if (gGrounded && drop > STEP_SNAP_EPS && drop <= STEP_DOWN_MAX) {
                playerFootY = newFloor;
                playerAbs.y = playerFootY + PLAYER_FOOT_BIAS;
            }
        }

        // Shooting
        gShootCooldown -= deltaTime;
        if (gShootHeld && gShootCooldown <= 0.0f) {
            float yaw = glm::radians(gPlayerYawDeg);
            glm::vec3 fwd = glm::normalize(glm::vec3(std::sin(yaw), 0.0f, -std::cos(yaw)));
            Bullet b; b.pos = playerAbs + glm::vec3(0, 0.5f, 0) + fwd * 0.9f;
            b.vel = fwd * 20.0f; b.life = 4.0f; b.radius = 0.2f; b.active = true;
            gBullets.push_back(b); gShootCooldown = 0.12f;
        }

        // Enemy patrolling (demo)
        enemyPosXZ.x += enemyDir * enemySpeed * deltaTime;
        if (enemyPosXZ.x > -6.f + enemyAmp) enemyDir = -1.f;
        if (enemyPosXZ.x < -6.f - enemyAmp) enemyDir = 1.f;

        // Item pickup
        itemBox.center = { itemPosXZ.x, itemPosXZ.z };
        if (!itemCollected && IntersectsXZ(playerBox, itemBox)) { itemCollected = true; gWalkSpeed = 12.0f; }

        // Bullet hits
        enemyBox.center = { enemyPosXZ.x, enemyPosXZ.z };
        for (auto& b : gBullets) if (b.active) {
            b.pos += b.vel * deltaTime;
            b.life -= deltaTime;
            if (b.life <= 0.0f) b.active = false;
            AABB2D bb{ {b.pos.x,b.pos.z},{b.radius,b.radius} };
            if (b.active && IntersectsXZ(bb, enemyBox)) { b.active = false; enemyHP--; }
        }
        gBullets.erase(std::remove_if(gBullets.begin(), gBullets.end(),
            [](const Bullet& b) { return !b.active; }), gBullets.end());
        if (enemyHP <= 0) { enemyHP = 7; enemyPosXZ = glm::vec3(-6.f, 0.f, 2.f); }

        // ---------- Render ----------
        glClearColor(0.06f, 0.07f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        shader.use();
        float farPlane = 600.0f;
        glm::mat4 P = glm::perspective(glm::radians(camera.Zoom),
            (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, farPlane);
        shader.setMat4("projection", P);
        shader.setMat4("view", camera.GetViewMatrix());

        updateFollowCamera(playerAbs);

        // Map
        {
            glm::mat4 M = MapTransform();
            shader.setMat4("model", M);
            mapModel.Draw(shader);
        }

        if (!itemCollected) drawAbs(itemModel, itemAbs, 0.0f, 0.85f);
        drawAbs(enemyModel, enemyAbs, t * 30.0f, enemyScale);
        drawAbs(playerModel, playerAbs, gPlayerYawDeg, playerScale);

        for (auto& b : gBullets) if (b.active) drawAbs(ballModel, b.pos, 0.0f, 0.025f);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}
