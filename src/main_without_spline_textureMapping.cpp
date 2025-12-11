#ifdef _WIN32
#include <windows.h>
#include <GL/glut.h> 
#else
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#endif

#ifdef _MSC_VER
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#endif
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>

// 텍스쳐 매핑을 위한 라이브러리
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"  // PNG, JPG 등 픽셀 데이터 읽어오는 헤더 !경로지정필요
#include <omp.h>

// --- 설정 변수 ---
const int numRays = 300; // 성능을 위해 1000 -> 300으로 조정 (테스트 후 늘리세요)
static float Time = 0.0f; // 정밀한 회전을 위해 float 변경

struct Body {
    glm::vec3 position;     // 현재 월드 상의 위치
    float mass;
    float radius;
    glm::vec3 color;

    // 궤도 정보
    Body* parent = nullptr; // 부모 천체 포인터
    glm::vec3 relativeOffset; // 부모로부터의 거리 (초기 오프셋)
    float orbitRadius;
    float orbitSpeed;       // 공전 속도
    float rotationSpeed;    // 자전 속도
};

// --- 전역 변수 ---
std::vector<std::vector<glm::vec3>> rayPaths(numRays);
std::vector<Body*> bodies;
float lightSpeed = 30.0f;
float dt = 0.01f; // 시뮬레이션 스텝 간격 조정

std::vector<glm::vec3> initialVelocities(numRays);
glm::vec4 lightPosition = { -15.0f, 10.0f, -10.0f, 1.0f };

// 태양 텍스쳐 매핑을 위한 변수 설정
GLuint sunTexture = 0;
bool sunTextureLoaded = false;
GLUquadric* sunQuadric = nullptr;

// 행성 텍스처(수성, 금성, 목성)와 쿼드릭
GLuint mercuryTexture = 0;
GLuint venusTexture = 0;
GLuint jupiterTexture = 0;
bool mercuryTextureLoaded = false;
bool venusTextureLoaded = false;
bool jupiterTextureLoaded = false;
GLUquadric* planetQuadric = nullptr;

// 스카이박스(6면) 텍스처
enum SkyboxFace {
    SKYBOX_RIGHT = 0,  // +X
    SKYBOX_LEFT = 1,  // -X
    SKYBOX_TOP = 2,  // +Y
    SKYBOX_BOTTOM = 3,  // -Y
    SKYBOX_FRONT = 4,  // +Z
    SKYBOX_BACK = 5   // -Z
};
GLuint skyboxTextures[6] = { 0, };
bool skyboxTexturesLoaded = false;

// 카메라 및 인터랙션
float cameraAngleX = 0.0f;
float cameraAngleY = 0.0f;
float cameraDistance = 80.0f;
int lastMouseX, lastMouseY;
bool isDragging = false;
int selectedBodyIndex = -1;

// Picking을 위한 행렬 저장소
GLdouble savedModelview[16];
GLdouble savedProjection[16];
GLint savedViewport[4];

// --- 함수 정의 ---

void setupScene() {
    // 1. 블랙홀 (중심)
    Body* blackhole = new Body();
    blackhole->position = { 0,0,0 };
    blackhole->mass = 800.0f; // 질량 키움
    blackhole->radius = 4.0f;
    blackhole->color = { 0.1f, 0.1f, 0.1f };
    blackhole->orbitSpeed = 0.0f;
    blackhole->rotationSpeed = 0.05f;
    blackhole->parent = nullptr;

    // 2. 중성자별 (블랙홀 주위를 공전)
    Body* neutronStar = new Body();
    neutronStar->mass = 500.0f;
    neutronStar->radius = 2.0f;
    neutronStar->color = { 0.4f, 0.4f, 0.9f };
    neutronStar->parent = blackhole;
    neutronStar->orbitRadius = 25.0f; // 거리
    neutronStar->orbitSpeed = 1.0f;   // 공전 속도
    neutronStar->rotationSpeed = 2.0f;

    // 3. 행성 (중성자별 주위를 공전)
    Body* planet1 = new Body();
    planet1->mass = 100.0f;
    planet1->radius = 1.0f;
    planet1->color = { 0.8f, 0.3f, 0.3f };
    planet1->parent = neutronStar;
    planet1->orbitRadius = 8.0f;
    planet1->orbitSpeed = 3.0f;
    planet1->rotationSpeed = 1.0f;

    bodies.push_back(blackhole);
    bodies.push_back(neutronStar);
    bodies.push_back(planet1);
}

// 공통 텍스처 로더
GLuint loadTextureGeneric(const char* filename) {
    int width, height, channels;
    unsigned char* data = stbi_load(filename, &width, &height, &channels, 0);
    if (!data) {
        std::cerr << "Failed to load texture: " << filename << std::endl;
        return 0;
    }

    GLuint texId = 0;
    glGenTextures(1, &texId);
    glBindTexture(GL_TEXTURE_2D, texId);

    GLenum format = GL_RGB;
    if (channels == 4) {
        format = GL_RGBA;
    }

    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    stbi_image_free(data);
    return texId;
}

// 태양 텍스쳐 로드 함수
bool loadSunTexture(const char* filename) {
    sunTexture = loadTextureGeneric(filename);
    if (sunTexture == 0) return false;
    sunTextureLoaded = true;
    return true;
}

bool loadMercuryTexture(const char* filename) {
    mercuryTexture = loadTextureGeneric(filename);
    if (mercuryTexture == 0) return false;
    mercuryTextureLoaded = true;
    return true;
}

bool loadVenusTexture(const char* filename) {
    venusTexture = loadTextureGeneric(filename);
    if (venusTexture == 0) return false;
    venusTextureLoaded = true;
    return true;
}

bool loadJupiterTexture(const char* filename) {
    jupiterTexture = loadTextureGeneric(filename);
    if (jupiterTexture == 0) return false;
    jupiterTextureLoaded = true;
    return true;
}

bool loadSkyboxTextures(const char* rightPath,
    const char* leftPath,
    const char* topPath,
    const char* bottomPath,
    const char* frontPath,
    const char* backPath) {
    const char* paths[6] = {
        rightPath, leftPath, topPath, bottomPath, frontPath, backPath
    };

    for (int i = 0; i < 6; ++i) {
        skyboxTextures[i] = loadTextureGeneric(paths[i]);
        if (skyboxTextures[i] == 0) {
            std::cerr << "Failed to load skybox texture: " << paths[i] << std::endl;
            return false;
        }
    }
    skyboxTexturesLoaded = true;
    return true;
}

void makeVelocities() {
    for (int i = 0; i < numRays; i++) {
        // 구면으로 랜덤하게 퍼지는 빛
        initialVelocities[i] = glm::sphericalRand(1.0f) * lightSpeed;
    }
}

// 순수 수학으로 위치 업데이트 (OpenGL 행렬 의존 X)
void updateBodyPhysics(float currentTime) {
    for (auto& body : bodies) {
        if (body->parent == nullptr) {
            // 중심 천체는 원점에 고정 (원한다면 이동 가능)
            body->position = glm::vec3(0.0f, 0.0f, 0.0f);
        }
        else {
            // 부모 기준으로 공전 계산
            float angle = currentTime * body->orbitSpeed * 0.5f; // 속도 조절
            float x = cos(angle) * body->orbitRadius;
            float z = sin(angle) * body->orbitRadius;

            // 부모 위치 + 공전 위치
            body->position = body->parent->position + glm::vec3(x, 0.0f, z);
        }
    }
}

void simulateRay(glm::vec3 startPos) {
    // 성능 최적화를 위해 매 프레임 벡터 재할당 방지 (크기만 유지)
    if (rayPaths.size() != numRays) rayPaths.resize(numRays);

    for (int i = 0; i < numRays; i++) {
        glm::vec3 pos = startPos;
        glm::vec3 vel = initialVelocities[i];

        std::vector<glm::vec3>& path = rayPaths[i];
        path.clear();
        path.reserve(500); // 메모리 예약
        path.push_back(pos);

        // 최대 스텝 수 감소 (성능 타협점)
        int maxSteps = 2000;

        for (int step = 0; step < maxSteps; step++) {
            glm::vec3 totalAccel = { 0, 0, 0 };
            bool crashed = false;
            float minDistSq = 1e9f;

            for (const auto& body : bodies) {
                glm::vec3 dir = body->position - pos;
                float distSq = glm::dot(dir, dir);

                if (distSq < body->radius * body->radius) {
                    crashed = true;
                    break;
                }

                if (distSq < minDistSq) minDistSq = distSq;

                // 중력 가속도 F = G * M / r^2 (G=1로 가정, 방향 벡터 정규화 포함)
                // a = M / r^2 * (dir / r) = M * dir / r^3
                float dist = sqrt(distSq);
                float accelMag = body->mass / (distSq * dist);
                totalAccel += dir * accelMag * 5.0f; // * 5.0f는 중력 효과 과장을 위한 계수
            }

            if (crashed) break;

            // 가변 dt (천체 근처에서는 정밀하게, 멀면 빠르게)
            float currentDt = dt;
            if (minDistSq > 500.0f) currentDt *= 2.0f;
            if (minDistSq > 2000.0f) currentDt *= 4.0f;

            vel += totalAccel * currentDt;
            pos += vel * currentDt;

            // 경계 체크
            if (abs(pos.x) > 200.0f || abs(pos.y) > 200.0f || abs(pos.z) > 200.0f) break;

            // 너무 촘촘하게 저장하면 그리기 느려짐, 일정 간격마다 저장
            if (step % 10 == 0) path.push_back(pos);
        }
        // 마지막 위치 저장
        path.push_back(pos);
    }
}

void initLighting() {
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    GLfloat light_pos_gl[] = { lightPosition.x, lightPosition.y, lightPosition.z, 1.0f };
    GLfloat white[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos_gl);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);
}

void drawSkybox() {
    if (!skyboxTexturesLoaded) return;

    float size = 300.0f; // 스카이박스 크기 (카메라 거리보다 충분히 크게)

    glDepthMask(GL_FALSE);      // 깊이 버퍼에 쓰지 않음
    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);

    // +X (right)
    glBindTexture(GL_TEXTURE_2D, skyboxTextures[SKYBOX_RIGHT]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(size, -size, -size);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(size, -size, size);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(size, size, size);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(size, size, -size);
    glEnd();

    // -X (left)
    glBindTexture(GL_TEXTURE_2D, skyboxTextures[SKYBOX_LEFT]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-size, -size, size);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-size, -size, -size);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-size, size, -size);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-size, size, size);
    glEnd();

    // +Y (top)
    glBindTexture(GL_TEXTURE_2D, skyboxTextures[SKYBOX_TOP]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-size, size, -size);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(size, size, -size);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(size, size, size);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-size, size, size);
    glEnd();

    // -Y (bottom)
    glBindTexture(GL_TEXTURE_2D, skyboxTextures[SKYBOX_BOTTOM]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-size, -size, size);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(size, -size, size);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(size, -size, -size);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-size, -size, -size);
    glEnd();

    // +Z (front)
    glBindTexture(GL_TEXTURE_2D, skyboxTextures[SKYBOX_FRONT]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-size, -size, size);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(size, -size, size);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(size, size, size);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-size, size, size);
    glEnd();

    // -Z (back)
    glBindTexture(GL_TEXTURE_2D, skyboxTextures[SKYBOX_BACK]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(size, -size, -size);
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-size, -size, -size);
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-size, size, -size);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(size, size, -size);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);
    glDepthMask(GL_TRUE);
}

void init() {
    glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    initLighting();
    setupScene();
    makeVelocities();

    // 태양 텍스처 및 쿼드릭 초기화
    if (!sunQuadric) {
        sunQuadric = gluNewQuadric();
        gluQuadricTexture(sunQuadric, GL_TRUE);
        gluQuadricNormals(sunQuadric, GLU_SMOOTH);
    }
    if (!planetQuadric) {
        planetQuadric = gluNewQuadric();
        gluQuadricTexture(planetQuadric, GL_TRUE);
        gluQuadricNormals(planetQuadric, GLU_SMOOTH);
    }

    // 텍스쳐 파일 로드
    glEnable(GL_TEXTURE_2D);
    if (!sunTextureLoaded) {
        if (!loadSunTexture("C:\\teamproject\\teamproject\\texture\\8k_sun.jpg")) {
            std::cerr << "Failed to load 2k_sun texture" << std::endl;
        }
    }
    if (!mercuryTextureLoaded) {
        if (!loadMercuryTexture("C:\\teamproject\\teamproject\\texture\\8k_mercury.jpg")) {
            std::cerr << "Failed to load 2k_mercury texture" << std::endl;
        }
    }
    if (!venusTextureLoaded) {
        if (!loadVenusTexture("C:\\teamproject\\teamproject\\texture\\8k_venus.jpg")) {
            std::cerr << "Failed to load 2k_venus texture" << std::endl;
        }
    }
    if (!jupiterTextureLoaded) {
        if (!loadJupiterTexture("C:\\teamproject\\teamproject\\texture\\8k_jupiter.jpg")) {
            std::cerr << "Failed to load 2k_jupiter texture" << std::endl;
        }
    }
    if (!skyboxTexturesLoaded) {
        // 실제 파일 경로/이름에 맞게 수정해서 사용
        if (!loadSkyboxTextures(
            "C:\\teamproject\\teamproject\\texture\\px.png", // 오른쪽
            "C:\\teamproject\\teamproject\\texture\\nx.png", // 왼쪽
            "C:\\teamproject\\teamproject\\texture\\py.png", // 위
            "C:\\teamproject\\teamproject\\texture\\ny.png", // 아래
            "C:\\teamproject\\teamproject\\texture\\pz.png", // 앞
            "C:\\teamproject\\teamproject\\texture\\nz.png")) // 뒤
        {
            std::cerr << "Failed to load skybox textures" << std::endl;
        }
    }
    glDisable(GL_TEXTURE_2D);
}

void drawScene() {
    // 1. 천체 그리기
    for (int i = 0; i < bodies.size(); ++i) {
        Body* b = bodies[i];
        glPushMatrix();
        glTranslatef(b->position.x, b->position.y, b->position.z);

        // 텍스처가 90도 누워 있어서 X축 기준으로 세워줌
        glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);

        // 자전 시각화
        glRotatef(Time * b->rotationSpeed * 50.0f, 0, 1, 0);

        // 선택된 천체 표시
        if (i == selectedBodyIndex) {
            glDisable(GL_LIGHTING);
            glColor3f(0.0f, 1.0f, 0.0f);
            glutWireSphere(b->radius * 1.5f, 16, 16);
            glEnable(GL_LIGHTING);
        }

        // 인덱스 기준으로 각 구체에 텍스처 매핑
        GLuint texId = 0;
        bool useTexture = false;
        if (i == 0 && mercuryTextureLoaded && mercuryTexture != 0) {
            texId = mercuryTexture;
            useTexture = true;
        }
        else if (i == 1 && venusTextureLoaded && venusTexture != 0) {
            texId = venusTexture;
            useTexture = true;
        }
        else if (i == 2 && jupiterTextureLoaded && jupiterTexture != 0) {
            texId = jupiterTexture;
            useTexture = true;
        }

        if (useTexture && planetQuadric != nullptr) {
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, texId);
            glColor3f(1.0f, 1.0f, 1.0f); // 텍스처 색 그대로 사용
            gluSphere(planetQuadric, b->radius, 32, 32);
            glDisable(GL_TEXTURE_2D);
        }
        else {
            glColor3fv(glm::value_ptr(b->color));
            glutSolidSphere(b->radius, 32, 32);
        }

        glPopMatrix();
    }

    // 2. 광선 그리기
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE); // Additive Blending (빛 효과)
    glLineWidth(1.2f);

    for (const auto& path : rayPaths) {
        glBegin(GL_LINE_STRIP);
        glColor4f(1.0f, 0.8f, 0.4f, 0.3f); // 반투명한 노란색
        for (const auto& p : path) {
            glVertex3f(p.x, p.y, p.z);
        }
        glEnd();
    }
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);

    // 3. 태양(광원) 구체 표시 - numRays가 뿜어져 나오는 중심
    glPushMatrix();
    glTranslatef(lightPosition.x, lightPosition.y, lightPosition.z);
    // 텍스처가 90도 누워 있어서 X축 기준으로 세워줌
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);

    // 태양 광원 효과
    glDisable(GL_LIGHTING);  // 스스로 빛나는 느낌
    if (sunTextureLoaded && sunTexture != 0 && sunQuadric != nullptr) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, sunTexture);
        glColor3f(1.0f, 1.0f, 1.0f); // 텍스처 색 그대로 사용

        gluSphere(sunQuadric, 10.0, 64, 64);

        glDisable(GL_TEXTURE_2D);
    }
    else {
        // 텍스처 로드 실패 시 색만 사용하는 구체
        glColor3f(1.0f, 0.9f, 0.3f);
        glutSolidSphere(3.0f, 32, 32);
    }
    glEnable(GL_LIGHTING);
    glPopMatrix();
}

void display() {
    // 1. 물리 업데이트 (CPU)
    Time += 0.02f;
    updateBodyPhysics(Time);
    simulateRay(glm::vec3(lightPosition));

    // 2. 렌더링 준비
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    float camX = cameraDistance * sin(cameraAngleY) * cos(cameraAngleX);
    float camY = cameraDistance * sin(cameraAngleX);
    float camZ = cameraDistance * cos(cameraAngleY) * cos(cameraAngleX);
    gluLookAt(camX, camY, camZ, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // 스카이박스: 카메라 위치 제거(회전만 유지)
    glPushMatrix();
    GLfloat mv[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, mv);
    mv[12] = mv[13] = mv[14] = 0.0f; // translation 제거
    glLoadMatrixf(mv);
    drawSkybox();
    glPopMatrix();

    // 3. Picking을 위해 현재 행렬 상태 저장 (중요!)
    glGetDoublev(GL_MODELVIEW_MATRIX, savedModelview);
    glGetDoublev(GL_PROJECTION_MATRIX, savedProjection);
    glGetIntegerv(GL_VIEWPORT, savedViewport);

    // 4. 그리기
    drawScene();

    glutSwapBuffers();
}

void pickBody(int mouseX, int mouseY) {
    selectedBodyIndex = -1;
    double minDepth = 1.0;

    // 저장해둔 행렬 사용
    for (int i = 0; i < bodies.size(); ++i) {
        double winX, winY, winZ;

        // 천체 중심 투영
        gluProject(bodies[i]->position.x, bodies[i]->position.y, bodies[i]->position.z,
            savedModelview, savedProjection, savedViewport,
            &winX, &winY, &winZ);

        // 천체 표면(반지름) 투영하여 화면상 크기 계산
        double edgeX, edgeY, edgeZ;
        gluProject(bodies[i]->position.x + bodies[i]->radius, bodies[i]->position.y, bodies[i]->position.z,
            savedModelview, savedProjection, savedViewport,
            &edgeX, &edgeY, &edgeZ);

        double radiusScreen = fabs(edgeX - winX);
        // 클릭 판정 범위에 여유를 줌 (최소 5픽셀)
        radiusScreen = std::max(radiusScreen, 5.0);

        double dist = sqrt(pow(mouseX - winX, 2) + pow((savedViewport[3] - mouseY) - winY, 2));

        if (dist <= radiusScreen && winZ < minDepth) {
            selectedBodyIndex = i;
            minDepth = winZ;
        }
    }

    if (selectedBodyIndex != -1) {
        std::cout << "Selected Body: " << selectedBodyIndex << " (Mass: " << bodies[selectedBodyIndex]->mass << ")" << std::endl;
    }
}

void mouseFunc(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            pickBody(x, y); // 저장된 행렬로 계산하는 함수 호출
            if (selectedBodyIndex == -1) {
                isDragging = true;
                lastMouseX = x;
                lastMouseY = y;
            }
        }
        else {
            isDragging = false;
        }
    }
    // 휠 줌인/아웃
    if (button == 3) cameraDistance -= 5.0f;
    if (button == 4) cameraDistance += 5.0f;
    if (cameraDistance < 10.0f) cameraDistance = 10.0f;
}

void motionFunc(int x, int y) {
    if (isDragging) {
        cameraAngleY += (x - lastMouseX) * 0.005f;
        cameraAngleX += (y - lastMouseY) * 0.005f;

        // 각도 제한
        if (cameraAngleX > 1.5f) cameraAngleX = 1.5f;
        if (cameraAngleX < -1.5f) cameraAngleX = -1.5f;

        lastMouseX = x;
        lastMouseY = y;
    }
}

void specialKeyFunc(int key, int x, int y) {
    if (selectedBodyIndex != -1) {
        if (key == GLUT_KEY_UP) bodies[selectedBodyIndex]->mass += 50.0f;
        if (key == GLUT_KEY_DOWN) {
            bodies[selectedBodyIndex]->mass -= 50.0f;
            if (bodies[selectedBodyIndex]->mass < 0) bodies[selectedBodyIndex]->mass = 0;
        }
        std::cout << "Body " << selectedBodyIndex << " Mass: " << bodies[selectedBodyIndex]->mass << std::endl;
    }
}

void MyTimer(int Value) {
    glutPostRedisplay();
    glutTimerFunc(16, MyTimer, 1); // 약 60 FPS 목표
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, (float)w / h, 1.0f, 500.0f);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1280, 720);
    glutCreateWindow("Gravitational Lensing Fixed");

    init();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseFunc);
    glutMotionFunc(motionFunc);
    glutSpecialFunc(specialKeyFunc);
    glutTimerFunc(16, MyTimer, 1);

    glutMainLoop();
    return 0;
}

