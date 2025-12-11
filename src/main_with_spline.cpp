#include <GL/glut.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <omp.h>
#include <glm/gtc/matrix_transform.hpp>

// --- 설정 변수 ---
const int numRays = 300;
static float Time = 0.0f;

struct Body {
    glm::vec3 position;
    float mass;
    float radius;
    glm::vec3 color;
    std::vector<Body*> satellites = {};
    
    Body* parent = nullptr;
    glm::vec3 relativeOffset;
    float orbitRadius;
    float orbitSpeed;
    float rotationSpeed;
};

// --- 전역 변수 ---
std::vector<std::vector<glm::vec3>> rayPaths(numRays);
std::vector<Body*> bodies;
float lightSpeed = 30.0f;

// [변경 1] 시뮬레이션 간격(dt)을 대폭 늘림. 계산 횟수를 줄이기 위함.
// 기존 0.05f -> 0.3f (너무 크면 정확도가 떨어지니 적절히 조절 필요)
float dt = 0.3f; 

std::vector<glm::vec3> initialVelocities(numRays);
glm::vec4 lightPosition = { -15.0f, 10.0f, -10.0f, 1.0f };

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

// --- [추가] Spline 함수 (Catmull-Rom) ---
// p0, p1, p2, p3 네 개의 점을 이용해 p1과 p2 사이의 곡선상 위치를 반환
glm::vec3 catmullRom(const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, float t) {
    float t2 = t * t;
    float t3 = t2 * t;

    // Catmull-Rom 공식 (GLM 벡터 연산 활용)
    // 0.5 * ( (2*p1) + (-p0 + p2)*t + (2*p0 - 5*p1 + 4*p2 - p3)*t^2 + (-p0 + 3*p1 - 3*p2 + p3)*t3 )
    glm::vec3 result = p1 * 2.0f;
    result += (p2 - p0) * t;
    result += (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2;
    result += (-p0 + p1 * 3.0f - p2 * 3.0f + p3) * t3;
    result *= 0.5f;

    return result;
}

// 점이 화면(Frustum) 안에 있는지 검사하는 함수
bool isPointVisible(const glm::vec3& point, const glm::mat4& mvpMatrix) {
    glm::vec4 p = mvpMatrix * glm::vec4(point, 1.0f);

    // OpenGL의 클립 공간 좌표 범위: -w <= x, y, z <= w
    // 약간의 여유(margin)를 주어 경계선에서 갑자기 사라지는 것을 방지 (1.0 -> 1.2 등)
    float w = p.w * 1.1f;

    return (p.x >= -w && p.x <= w) &&
        (p.y >= -w && p.y <= w) &&
        (p.z >= -w && p.z <= w);
}


// --- 함수 정의 ---

void setupScene() {
    // 1. 블랙홀 (중심)
    Body* blackhole = new Body();
    blackhole->position = { 0,0,0 };
    blackhole->mass = 800.0f;
    blackhole->radius = 4.0f;
    blackhole->color = { 0.1f, 0.1f, 0.1f };
    blackhole->orbitSpeed = 0.0f;
    blackhole->rotationSpeed = 0.05f;
    blackhole->parent = nullptr;

    // 2. 중성자별
    Body* neutronStar = new Body();
    neutronStar->mass = 500.0f;
    neutronStar->radius = 2.0f;
    neutronStar->color = { 0.4f, 0.4f, 0.9f };
    neutronStar->parent = blackhole;
    neutronStar->orbitRadius = 25.0f;
    neutronStar->orbitSpeed = 1.0f;
    neutronStar->rotationSpeed = 2.0f;

    // 3. 행성
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

void makeVelocities() {
    for (int i = 0; i < numRays; i++) {
        initialVelocities[i] = glm::sphericalRand(1.0f) * lightSpeed;
    }
}

void updateBodyPhysics(float currentTime) {
    for (auto& body : bodies) {
        if (body->parent == nullptr) {
            body->position = glm::vec3(0.0f, 0.0f, 0.0f);
        }
        else {
            float angle = currentTime * body->orbitSpeed * 0.5f;
            float x = cos(angle) * body->orbitRadius;
            float z = sin(angle) * body->orbitRadius;
            body->position = body->parent->position + glm::vec3(x, 0.0f, z);
        }
    }
}

void simulateRay(glm::vec3 startPos) {
    if (rayPaths.size() != numRays) rayPaths.resize(numRays);

    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < numRays; i++) {
        glm::vec3 pos = startPos;
        glm::vec3 vel = initialVelocities[i];
        
        std::vector<glm::vec3>& path = rayPaths[i];
        path.clear();
        path.reserve(200); // 예약 크기 감소
        path.push_back(pos);

        // [변경 2] 최대 스텝 수를 대폭 줄임.
        // 기존 2000 -> 200. dt가 커졌으므로 더 적은 스텝으로 먼 거리를 감.
        int maxSteps = 2000; 
        
        for (int step = 0; step < maxSteps; step++) {
            glm::vec3 totalAccel = { 0, 0, 0 };
            bool crashed = false;
            float minDistSq = 1e9f;

            for (const auto& body : bodies) {
                glm::vec3 dir = body->position - pos;
                float distSq = glm::dot(dir, dir);
                
                if (distSq < body->radius * body->radius) {
                    crashed = true; break;
                }
                if(distSq < minDistSq) minDistSq = distSq;

                float dist = sqrt(distSq);
                // 거리 안전장치 추가 (너무 가까우면 가속도 폭발 방지)
                float safeDistSq = std::max(distSq, 1.0f);
                float safeDist = sqrt(safeDistSq);

                float accelMag = body->mass / (safeDistSq * safeDist); 
                totalAccel += dir * accelMag * 5.0f;
            }

            if (crashed) break;

            // 가변 dt 적용 (멀리 있을 땐 더 빠르게 계산)
            float currentDt = dt;
            if (minDistSq > 1000.0f) currentDt *= 2.0f;
            if (minDistSq > 5000.0f) currentDt *= 4.0f;

            vel += totalAccel * currentDt;
            pos += vel * currentDt;

            if (abs(pos.x) > 300.0f || abs(pos.y) > 300.0f || abs(pos.z) > 300.0f) break;

            // [중요] 스플라인을 위해 모든 계산된 점을 저장합니다.
            // 점들 사이의 간격이 넓으므로 다 저장해도 개수가 많지 않습니다.
            path.push_back(pos);
        }
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

void init() {
    glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH); // 선 부드럽게
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    initLighting();
    setupScene();
    makeVelocities();
}

void drawScene() {
    // 1. MVP 행렬 미리 계산 (루프 밖에서 한 번만 수행하여 성능 확보)
    // savedProjection과 savedModelview는 display()에서 저장해둔 것 사용
    glm::mat4 projMat = glm::make_mat4(savedProjection);
    glm::mat4 viewMat = glm::make_mat4(savedModelview);
    glm::mat4 mvpMat = projMat * viewMat; // Model-View-Projection

    // --- 천체 그리기 (기존과 동일) ---
    for (int i = 0; i < bodies.size(); ++i) {
        Body* b = bodies[i];

        // [추가 최적화] 천체 자체도 화면 밖이면 안 그림 (선택적)
        if (!isPointVisible(b->position, mvpMat)) continue;

        glPushMatrix();
        glTranslatef(b->position.x, b->position.y, b->position.z);
        glRotatef(Time * b->rotationSpeed * 50.0f, 0, 1, 0);

        if (i == selectedBodyIndex) {
            glDisable(GL_LIGHTING);
            glColor3f(0.0f, 1.0f, 0.0f);
            glutWireSphere(b->radius * 1.5f, 16, 16);
            glEnable(GL_LIGHTING);
        }

        glColor3fv(glm::value_ptr(b->color));
        glutSolidSphere(b->radius, 32, 32);
        glPopMatrix();
    }

    // --- 광선 그리기 (컬링 적용) ---
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    // 선 굵기 조절 (이전 질문 반영)
    glLineWidth(0.8f);

    for (const auto& path : rayPaths) {
        if (path.size() < 4) continue;

        glBegin(GL_LINE_STRIP);
        glColor4f(1.0f, 0.8f, 0.4f, 0.3f); // 투명도 조절

        for (size_t i = 0; i < path.size() - 1; ++i) {
            glm::vec3 p1 = path[i];
            glm::vec3 p2 = path[i + 1];

            // [핵심] 컬링 체크
            // 선분의 시작점(p1)과 끝점(p2)이 둘 다 화면 밖에 있다면 스킵!
            // 하나라도 안에 있다면(혹은 걸쳐 있다면) 그려야 함.
            bool v1 = isPointVisible(p1, mvpMat);
            bool v2 = isPointVisible(p2, mvpMat);

            if (!v1 && !v2) {
                // 둘 다 화면 밖이면 그리지 않고 다음 선분으로 넘어감.
                // 단, GL_LINE_STRIP 흐름이 끊기면 안 되므로 
                //glEnd(); glBegin(GL_LINE_STRIP); 처리를 해줘야 완벽하지만
                // 여기서는 성능을 위해 그냥 투명한 선을 긋거나 건너뛰는 방식을 씁니다.

                // 가장 간단한 방법: 건너뛰기 (glEnd/glBegin으로 끊어주면 더 좋음)
                glEnd();
                glBegin(GL_LINE_STRIP);
                continue;
            }

            // --- 스플라인 계산 및 그리기 ---
            glm::vec3 p0 = (i == 0) ? path[0] : path[i - 1];
            glm::vec3 p3 = (i + 1 == path.size() - 1) ? path[i + 1] : path[i + 2];

            int segments = 10;
            for (int j = 0; j <= segments; ++j) {
                float t = (float)j / (float)segments;
                glm::vec3 interpolatedPos = catmullRom(p0, p1, p2, p3, t);
                glVertex3fv(glm::value_ptr(interpolatedPos));
            }
        }
        glEnd();
    }
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);

    // 3. 광원 위치 표시
    glPushMatrix();
    glTranslatef(lightPosition.x, lightPosition.y, lightPosition.z);
    glDisable(GL_LIGHTING);
    glColor3f(1.0f, 1.0f, 1.0f);
    glutSolidSphere(0.5f, 10, 10);
    glEnable(GL_LIGHTING);
    glPopMatrix();
}

void display() {
    Time += 0.02f;
    updateBodyPhysics(Time);
    simulateRay(glm::vec3(lightPosition));

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    float camX = cameraDistance * sin(cameraAngleY) * cos(cameraAngleX);
    float camY = cameraDistance * sin(cameraAngleX);
    float camZ = cameraDistance * cos(cameraAngleY) * cos(cameraAngleX);
    gluLookAt(camX, camY, camZ, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    glGetDoublev(GL_MODELVIEW_MATRIX, savedModelview);
    glGetDoublev(GL_PROJECTION_MATRIX, savedProjection);
    glGetIntegerv(GL_VIEWPORT, savedViewport);

    drawScene();

    glutSwapBuffers();
}

void pickBody(int mouseX, int mouseY) {
    selectedBodyIndex = -1;
    double minDepth = 1.0;

    for (int i = 0; i < bodies.size(); ++i) {
        double winX, winY, winZ;
        gluProject(bodies[i]->position.x, bodies[i]->position.y, bodies[i]->position.z,
            savedModelview, savedProjection, savedViewport,
            &winX, &winY, &winZ);

        double edgeX, edgeY, edgeZ;
        gluProject(bodies[i]->position.x + bodies[i]->radius, bodies[i]->position.y, bodies[i]->position.z,
            savedModelview, savedProjection, savedViewport,
            &edgeX, &edgeY, &edgeZ);

        double radiusScreen = fabs(edgeX - winX);
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
            pickBody(x, y);
            if (selectedBodyIndex == -1) {
                isDragging = true;
                lastMouseX = x;
                lastMouseY = y;
            }
        } else {
            isDragging = false;
        }
    }
    if (button == 3) cameraDistance -= 5.0f;
    if (button == 4) cameraDistance += 5.0f;
    if (cameraDistance < 10.0f) cameraDistance = 10.0f;
}

void motionFunc(int x, int y) {
    if (isDragging) {
        cameraAngleY += (x - lastMouseX) * 0.005f;
        cameraAngleX += (y - lastMouseY) * 0.005f;
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
    glutTimerFunc(16, MyTimer, 1);
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, (float)w / h, 1.0f, 1000.0f); // Far plane 늘림
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE); // 멀티샘플링 추가 (선 부드럽게)
    glutInitWindowSize(1280, 720);
    glutCreateWindow("Gravitational Lensing with Splines");

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