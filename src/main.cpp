#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cmath> // 用于 sqrt
#include <limits> // 用于 std::numeric_limits

#include "Pre_process.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// ---------- 全局 ----------
mjModel* m = nullptr;
mjData* d = nullptr;

// 主场景和相机
mjvScene scn;
mjvCamera cam;
mjvOption opt;
mjrContext con;



// ---------- 用于机器人视角和深度/彩色图的全局变量 ----------
mjvScene scn_robot;
const int CAM_WIDTH = 320;
const int CAM_HEIGHT = 240;

// 缓冲区
unsigned char* rgb_buffer = nullptr;
unsigned char* depth_grayscale_buffer = nullptr;
float* depth_buffer = nullptr;

// OpenGL 纹理ID
GLuint depth_tex_id;
GLuint rgb_tex_id;


// ---------- 交互用全局 ----------
bool btn_l = false, btn_m = false, btn_r = false;
double lastx = 0, lasty = 0;
const float MOUSE_SENS  = 0.02f;
const float SCROLL_SENS = 0.2f;

// ---------- 用户输入状态 ----------
int  user_mode = 1;
bool walk_cmd  = false;

// ---------- 关节角度控制接口 ----------
bool set_joint_deg(const std::string &act_name, double deg){
    int act_id = mj_name2id(m, mjOBJ_ACTUATOR, act_name.c_str());
    if(act_id == -1){
        std::cerr << "[WARN] actuator '" << act_name << "' not found!\n";
        return false;
    }
    d->ctrl[act_id] = deg * M_PI / 180.0;
    return true;
}

// ---------- 辅助函数：将深度图（float）转换为可视化的灰度图（uchar） ----------
void convert_depth_to_grayscale(unsigned char* grayscale_buf, const float* depth_buf, int width, int height) {
    float min_val = 1.0f, max_val = 0.0f;
    for (int i = 0; i < width * height; ++i) {
        if (depth_buf[i] < 1.0f) {
            if (depth_buf[i] < min_val) min_val = depth_buf[i];
            if (depth_buf[i] > max_val) max_val = depth_buf[i];
        }
    }

    float range = max_val - min_val;
    if (range < 1e-5) {
        range = 1.0f;
        min_val = 0.0f;
    }

    for (int i = 0; i < width * height; ++i) {
        float z = depth_buf[i];
        unsigned char gray;
        if (z >= 1.0f) {
            gray = 0;
        } else {
            float normalized = (z - min_val) / range;
            gray = 255 - (unsigned char)(255.0f * std::max(0.0f, std::min(1.0f, normalized)));
        }
        grayscale_buf[3*i + 0] = gray;
        grayscale_buf[3*i + 1] = gray;
        grayscale_buf[3*i + 2] = gray;
    }
}

// ---------- 辅助函数：将深度图转换为PCL点云 (已修正) ----------
// 这个函数现在将MuJoCo的深度图和相机位姿转换为世界坐标系下的点云
void convertDepthToPointCloud(
    const float* depth_buf, int width, int height,
    const mjvCamera* cam, const mjModel* m, const mjData* d,
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& point_cloud)
{
    point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    point_cloud->width = width;
    point_cloud->height = height;
    point_cloud->is_dense = false;
    point_cloud->points.resize(width * height);

    // 获取视场角和裁剪面距离
    float fovy = m->vis.global.fovy;

    float extent = m->stat.extent;
    float znear = m->vis.map.znear ;
    float zfar = m->vis.map.zfar;
    // Get the actual znear and zfar used by the renderer for this camera
    // float znear = m->cam_fovy[cam->fixedcamid] > 0 ?
    //               m->vis.map.znear * m->stat.extent :
    //               -m->cam_pos[cam->fixedcamid * 3 + 2] - m->vis.map.znear;
    // float zfar = m->cam_fovy[cam->fixedcamid] > 0 ?
    //              m->vis.map.zfar * m->stat.extent :
    //              -m->cam_pos[cam->fixedcamid * 3 + 2] - m->vis.map.zfar;


    // 获取相机在世界坐标系下的位姿
    mjtNum cam_pos[3];
    mjtNum cam_mat[9];
    mju_copy3(cam_pos, &d->cam_xpos[cam->fixedcamid * 3]);
    mju_copy(cam_mat, &d->cam_xmat[cam->fixedcamid * 9], 9);

    // 遍历每一个像素
    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            // 1. 从深度缓冲区获取归一化的深度值 (0-1)
            float depth = depth_buf[r * width + c];

            // 如果深度值是1.0, 表示是背景或超出远裁剪面, 跳过
            if (depth >= 1.0f) {
                point_cloud->at(c, r).x = std::numeric_limits<float>::quiet_NaN();
                point_cloud->at(c, r).y = std::numeric_limits<float>::quiet_NaN();
                point_cloud->at(c, r).z = std::numeric_limits<float>::quiet_NaN();
                continue;
            }

            // 2. 将归一化的深度值转换回相机坐标系下的线性Z值
            // 参照OpenCV版本的深度线性化公式：z_near * z_far * extent / (z_far - raw_depth * (z_far - z_near))
            float z_cam = -znear * zfar * extent / (zfar - depth * (zfar - znear));

            // 3. 将像素坐标(c, r)转换为相机坐标系下的X和Y值
            float aspect = (float)width / (float)height;
            float tan_fovy_half = tan(fovy * M_PI / 360.0);
            float y_cam =  -tan_fovy_half * z_cam * (2.0f * r / (height-1) - 1.0f);
            
            float x_cam = aspect * tan_fovy_half * z_cam * (2.0f * c / (width-1) - 1.0f);

            // 4. 将相机坐标系下的点(x_cam, y_cam, z_cam)转换到世界坐标系
            mjtNum point_cam[3] = {x_cam, y_cam, z_cam};
            mjtNum point_world[3];
            mju_mulMatVec(point_world, cam_mat, point_cam, 3, 3);
            mju_addTo3(point_world, cam_pos);

            // 5. 存储世界坐标系下的点
            point_cloud->at(c, r).x = point_world[0];
            point_cloud->at(c, r).y = point_world[1];
            point_cloud->at(c, r).z = point_world[2];
        }
    }
}


// ---------- 辅助函数：绘制2D纹理覆盖层 (用于RGB和灰度深度图) ----------
void draw_texture_overlay(int window_width, int window_height, GLuint tex_id, int corner) {
    mjrRect overlay_rect;

    if (corner == 0) { // Top-Right for Depth
        overlay_rect = {window_width - CAM_WIDTH - 20, window_height - CAM_HEIGHT - 20, CAM_WIDTH, CAM_HEIGHT};
    } else { // Bottom-Right for RGB
        overlay_rect = {window_width - CAM_WIDTH - 20, 20, CAM_WIDTH, CAM_HEIGHT};
    }

    glPushAttrib(GL_ENABLE_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, window_width, 0, window_height, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex_id);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glBegin(GL_QUADS);
    glTexCoord2f(1, 1); glVertex2i(overlay_rect.left, overlay_rect.bottom);
    glTexCoord2f(0, 1); glVertex2i(overlay_rect.left + overlay_rect.width, overlay_rect.bottom);
    glTexCoord2f(0, 0); glVertex2i(overlay_rect.left + overlay_rect.width, overlay_rect.bottom + overlay_rect.height);
    glTexCoord2f(1, 0); glVertex2i(overlay_rect.left, overlay_rect.bottom + overlay_rect.height);
    glEnd();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glPopAttrib();
}


// ---------- 新增辅助函数：根据距离获取颜色（彩虹色谱）----------
void get_color_from_distance(float& r, float& g, float& b, float distance, float min_dist, float max_dist) {
    if (min_dist >= max_dist) {
        r = 0.5f; g = 0.5f; b = 0.5f; // 如果范围无效，则为灰色
        return;
    }

    // 将距离归一化到 [0, 1] 范围
    float normalized_dist = (distance - min_dist) / (max_dist - min_dist);
    normalized_dist = std::max(0.0f, std::min(1.0f, normalized_dist));

    // 简单的彩虹色谱 (Blue -> Green -> Red)
    r = 0.0f; g = 0.0f; b = 0.0f;
    if (normalized_dist < 0.5f) { // Blue -> Green
        float p = normalized_dist * 2.0f;
        g = p;
        b = 1.0f - p;
    } else { // Green -> Red
        float p = (normalized_dist - 0.5f) * 2.0f;
        r = p;
        g = 1.0f - p;
    }
}

// 只渲染点云，不包含场景背景
void draw_point_cloud_overlay(int window_width, int window_height,
                              mjvScene* robot_scn, mjrContext* main_con,
                              const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& cloud,
                              float min_dist, float max_dist)
{
    if (!cloud || cloud->empty()) {
        return;
    }

    mjrRect overlay_rect = {20, window_height - CAM_HEIGHT - 20, CAM_WIDTH, CAM_HEIGHT};
    
    // 设置视口但不渲染场景背景
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glViewport(overlay_rect.left, overlay_rect.bottom, overlay_rect.width, overlay_rect.height);

    // 清除视口区域为黑色背景
    glScissor(overlay_rect.left, overlay_rect.bottom, overlay_rect.width, overlay_rect.height);
    glEnable(GL_SCISSOR_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // 黑色背景
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);

    // 设置投影矩阵以匹配机器人摄像头视角
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    
    // 使用机器人摄像头的投影参数
    float fovy = 45;
    float aspect = (float)CAM_WIDTH / (float)CAM_HEIGHT;
    float znear = 0.01f;
    float zfar = 100.0f;
    gluPerspective(fovy, aspect, znear, zfar);

    // 设置模型视图矩阵
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    // 使用机器人摄像头的视角
    float* cam_pos = robot_scn->camera[0].pos;
    float* cam_forward = robot_scn->camera[0].forward;
    float* cam_up = robot_scn->camera[0].up;
    
    gluLookAt(cam_pos[0], cam_pos[1], cam_pos[2],
              cam_pos[0] + cam_forward[0], cam_pos[1] + cam_forward[1], cam_pos[2] + cam_forward[2],
              cam_up[0], cam_up[1], cam_up[2]);

    // 渲染点云
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glPointSize(3.0f);

    glBegin(GL_POINTS);
    for (const auto& point : cloud->points) {
        if (pcl::isFinite(point)) {
            float dx = point.x - cam_pos[0];
            float dy = point.y - cam_pos[1];
            float dz = point.z - cam_pos[2];
            float dist_to_cam = sqrt(dx*dx + dy*dy + dz*dz);

            float r, g, b;
            get_color_from_distance(r, g, b, dist_to_cam, min_dist, max_dist);
            glColor4f(r, g, b, 0.9f);
            glVertex3f(point.x, point.y, point.z);
        }
    }
    glEnd();

    // 恢复矩阵状态
    glPopMatrix(); // ModelView
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glPopAttrib();
    glViewport(0, 0, window_width, window_height);
}


// ---------- GLFW 回调 ----------
void mouse_btn_cb(GLFWwindow* window, int button, int action, int mods) {
    btn_l = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
    btn_m = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    btn_r = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move_cb(GLFWwindow* window, double xpos, double ypos) {
    if(!btn_l && !btn_m && !btn_r) return;
    float dx = float(xpos - lastx) * MOUSE_SENS;
    float dy = float(ypos - lasty) * MOUSE_SENS;
    mjtMouse act;
    if(btn_r)      act = mjMOUSE_ZOOM;
    else if(btn_l) act = mjMOUSE_ROTATE_H;
    else           act = mjMOUSE_MOVE_H;
    mjv_moveCamera(m, act, dx, dy, &scn, &cam);
    lastx = xpos; lasty = ypos;
}

void scroll_cb(GLFWwindow* window, double, double sy) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, float(sy) * SCROLL_SENS, &scn, &cam);
}
void key_cb(GLFWwindow*, int key, int, int action, int){
    if(action == GLFW_PRESS && key == GLFW_KEY_R){
        mj_resetData(m, d);
        std::cout << "[INPUT] Reset simulation\n";
    }
}

int main(int argc, char** argv){
    const char* xml_path = "/home/ziyi/workspace/mujoco_RGBD/model/scene/scene.xml"; // 默认路径
    if (argc > 1) {
        xml_path = argv[1];
    } else {
        printf("Usage: ./main <path_to_model.xml>\n");
        printf("Using default path: %s\n", xml_path);
    }

    // 初始化点云处理模块
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointCloud;
    Pre_process cloud_processor(pointCloud);

    std::cout << "Loading model from: " << xml_path << std::endl;

    char err[1000];
    m = mj_loadXML(xml_path, nullptr, err, 1000);
    if(!m){
        std::cerr << "XML load error: " << err << "\n";
        return 1;
    }
    d = mj_makeData(m);

    if (!glfwInit()) return -1;
    GLFWwindow* win = glfwCreateWindow(1200, 900, "MuJoCo Simulation with Point Cloud Visualization", nullptr, nullptr);
    if (!win) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);

    glfwSetMouseButtonCallback(win, mouse_btn_cb);
    glfwSetCursorPosCallback(  win, mouse_move_cb);
    glfwSetScrollCallback(     win, scroll_cb);
    glfwSetKeyCallback(        win, key_cb);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    mjv_makeScene(m, &scn_robot, 2000);
    mjr_resizeOffscreen(CAM_WIDTH, CAM_HEIGHT, &con);
    rgb_buffer = new unsigned char[CAM_WIDTH * CAM_HEIGHT * 3];
    depth_grayscale_buffer = new unsigned char[CAM_WIDTH * CAM_HEIGHT * 3];
    depth_buffer = new float[CAM_WIDTH * CAM_HEIGHT];

    glGenTextures(1, &depth_tex_id);
    glBindTexture(GL_TEXTURE_2D, depth_tex_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glGenTextures(1, &rgb_tex_id);
    glBindTexture(GL_TEXTURE_2D, rgb_tex_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();

    int robot_cam_id = mj_name2id(m, mjOBJ_CAMERA, "depth_cam");
    if (robot_cam_id == -1) {
        std::cerr << "Error: Camera 'depth_cam' not found in model." << std::endl;
        return -1;
    }

    while(!glfwWindowShouldClose(win)){
        auto now = clock::now();
        double realtime = std::chrono::duration<double>(now - t0).count();

        while(d->time < realtime){
            mj_step(m, d);
        }

        // 1. 从机器人摄像头渲染并获取深度/RGB数据
        mjvCamera cam_robot;
        cam_robot.type = mjCAMERA_FIXED;
        cam_robot.fixedcamid = robot_cam_id;
        mjv_updateScene(m, d, &opt, nullptr, &cam_robot, mjCAT_ALL, &scn_robot);

        mjrRect offscreen_vp = {0, 0, CAM_WIDTH, CAM_HEIGHT};
        mjr_setBuffer(mjFB_OFFSCREEN, &con);
        mjr_render(offscreen_vp, &scn_robot, &con);

        mjr_readPixels(rgb_buffer, depth_buffer, offscreen_vp, &con);

        // 2. 将深度图转换为世界坐标系下的点云
        convertDepthToPointCloud(depth_buffer, CAM_WIDTH, CAM_HEIGHT, &cam_robot, m, d, pointCloud);

        // 3. 计算点云的最小和最大距离以用于颜色映射
        float min_dist_raw = 1e6, max_dist_raw = -1e6;
        if (pointCloud && !pointCloud->empty()) {
            for (const auto& point : pointCloud->points) {
                 if (pcl::isFinite(point)) {
                    float dist_to_origin = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                    if (dist_to_origin < min_dist_raw) min_dist_raw = dist_to_origin;
                    if (dist_to_origin > max_dist_raw) max_dist_raw = dist_to_origin;
                }
            }
        }

        // 4. (可选) 使用您的算法处理点云
        // 输出机器人位置信息用于分析
        std::cout << "Robot position: [" << d->qpos[0] << ", " << d->qpos[1] << ", " << d->qpos[2] << "]" << std::endl;
        std::cout << "Camera position: [" << d->cam_xpos[robot_cam_id * 3] << ", " 
                  << d->cam_xpos[robot_cam_id * 3 + 1] << ", " 
                  << d->cam_xpos[robot_cam_id * 3 + 2] << "]" << std::endl;
        
        cloud_processor.pre_process(pointCloud);

        // 5. 渲染主场景
        mjr_setBuffer(mjFB_WINDOW, &con);
        int w, h;
        glfwGetFramebufferSize(win, &w, &h);
        mjrRect main_vp{0, 0, w, h};
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(main_vp, &scn, &con);

        // 6. 准备并渲染右侧的2D覆盖图
        convert_depth_to_grayscale(depth_grayscale_buffer, depth_buffer, CAM_WIDTH, CAM_HEIGHT);
        glBindTexture(GL_TEXTURE_2D, depth_tex_id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, CAM_WIDTH, CAM_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_grayscale_buffer);

        glBindTexture(GL_TEXTURE_2D, rgb_tex_id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, CAM_WIDTH, CAM_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb_buffer);

        draw_texture_overlay(w, h, depth_tex_id, 0); // 右上角
        draw_texture_overlay(w, h, rgb_tex_id, 1);   // 右下角

        // 7. 渲染左上角的3D点云覆盖图
        draw_point_cloud_overlay(w, h, &scn, &con, pointCloud, min_dist_raw, max_dist_raw);

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    // 清理资源
    delete[] rgb_buffer;
    delete[] depth_grayscale_buffer;
    delete[] depth_buffer;
    glDeleteTextures(1, &depth_tex_id);
    glDeleteTextures(1, &rgb_tex_id);
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjv_freeScene(&scn_robot);
    mjr_freeContext(&con);
    glfwTerminate();
    return 0;
}
