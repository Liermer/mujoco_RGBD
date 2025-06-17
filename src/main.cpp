#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include "Pre_process.hpp"

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
    
    static int frame_count = 0;
    // if (frame_count++ % 100 == 0) {
    //     printf("Depth range: min=%.3f, max=%.3f\n", min_val, max_val);
    // }

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
struct Point3D {
    float x, y, z;
};
void convertDepthToPointCloud(
    const float* depth_buf, int width, int height,
    float fx, float fy, float cx, float cy,
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& point_cloud)
{
    // point_cloud.clear();
    point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    point_cloud->reserve(width * height);

    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            float depth_value = depth_buf[v * width + u];

            // 深度值 < 1.0f 通常表示有效的物体距离，
            // >= 1.0f 表示背景或超出远裁剪平面的点，我们将其忽略。
            if (depth_value < 1.0f) {
                pcl::PointXYZ point;
                point.z = depth_value;
                point.x = (static_cast<float>(u) - cx) * point.z / fx;
                point.y = (static_cast<float>(v) - cy) * point.z / fy;
                
                point_cloud->push_back(point);
            }
        }
    }
}

// ---------- 改进的辅助函数：绘制覆盖层 ----------
void draw_overlay(int window_width, int window_height, int corner) {
    mjrRect overlay_rect;
    GLuint tex_id;

    if (corner == 0) { // Top-Right for Depth
        overlay_rect = {window_width - CAM_WIDTH - 20, window_height - CAM_HEIGHT - 20, CAM_WIDTH, CAM_HEIGHT};
        tex_id = depth_tex_id;
    } else { // Bottom-Right for RGB
        overlay_rect = {window_width - CAM_WIDTH - 20, 20, CAM_WIDTH, CAM_HEIGHT};
        tex_id = rgb_tex_id;
    }
    
    // 使用 glPush/PopAttrib 保存和恢复OpenGL状态，避免干扰3D渲染
    // glPushAttrib(GL_ENABLE_BIT | GL_PROJECTION_BIT | GL_MODELVIEW_BIT);
    glPushAttrib(GL_ENABLE_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, window_width, 0, window_height, -1, 1);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    // ***** 关键修复：禁用光照和深度测试 *****
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex_id);

    // 设置纹理的颜色如何与基本颜色混合
    // GL_REPLACE 会忽略基本颜色，只显示纹理
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glBegin(GL_QUADS);
    glTexCoord2f(1, 1); glVertex2i(overlay_rect.left, overlay_rect.bottom);
    glTexCoord2f(0, 1); glVertex2i(overlay_rect.left + overlay_rect.width, overlay_rect.bottom);
    glTexCoord2f(0, 0); glVertex2i(overlay_rect.left + overlay_rect.width, overlay_rect.bottom + overlay_rect.height);
    glTexCoord2f(1, 0); glVertex2i(overlay_rect.left, overlay_rect.bottom + overlay_rect.height);
    glEnd();
    
    // 恢复状态
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glPopAttrib();
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
    const char* xml_path = "/home/ziyi/workspace/mujoco_RGBD/model/scene/scene.xml";
    if (argc > 1) {
        xml_path = argv[1];
    }
    // 计算相机参数
    const double fovy_deg = 45.0; // 从 XML 中获取
    const double fovy_rad = fovy_deg * M_PI / 180.0;
    const float fx = static_cast<float>((CAM_HEIGHT / 2.0) / tan(fovy_rad / 2.0));
    const float fy = fx;
    const float cx = CAM_WIDTH / 2.0f;
    const float cy = CAM_HEIGHT / 2.0f;

    // ---------- 点云 ----------
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointCloud;
    Pre_process cloud(pointCloud);




    std::cout << "Loading model from: " << xml_path << std::endl;

    char err[1000];
    m = mj_loadXML(xml_path, nullptr, err, 1000);
    if(!m){
        std::cerr << "XML load error: " << err << "\n";
        return 1;
    }
    d = mj_makeData(m);

    if (!glfwInit()) return -1;
    GLFWwindow* win = glfwCreateWindow(1200, 900, "MuJoCo Exo with RGB/Depth View", nullptr, nullptr);
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
        
        mjvCamera cam_robot;
        cam_robot.type = mjCAMERA_FIXED;
        cam_robot.fixedcamid = robot_cam_id;
        mjv_updateScene(m, d, &opt, nullptr, &cam_robot, mjCAT_ALL, &scn_robot);
        
        mjrRect offscreen_vp = {0, 0, CAM_WIDTH, CAM_HEIGHT};
        mjr_setBuffer(mjFB_OFFSCREEN, &con);
        mjr_render(offscreen_vp, &scn_robot, &con);
        
        mjr_readPixels(rgb_buffer, depth_buffer, offscreen_vp, &con);

        // 将深度图转换为点云
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointCloud;
        convertDepthToPointCloud(depth_buffer, CAM_WIDTH, CAM_HEIGHT, fx, fy, cx, cy, pointCloud);
        
        // (可选) 打印一些点云信息来验证
        // static int frame_count_pc = 0;
        // if (frame_count_pc++ % 100 == 0 && !pointCloud.empty()) {
        //     printf("Point cloud generated with %zu points.\n", pointCloud.size());
        //     printf("First point: (%.3f, %.3f, %.3f)\n", 
        //            pointCloud[0].x, pointCloud[0].y, pointCloud[0].z);
        // }

        cloud.pre_process(pointCloud);

        // 渲染主场景
        mjr_setBuffer(mjFB_WINDOW, &con);
        int w, h;
        glfwGetFramebufferSize(win, &w, &h);
        mjrRect main_vp{0, 0, w, h};
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(main_vp, &scn, &con);
        
        convert_depth_to_grayscale(depth_grayscale_buffer, depth_buffer, CAM_WIDTH, CAM_HEIGHT);
        glBindTexture(GL_TEXTURE_2D, depth_tex_id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, CAM_WIDTH, CAM_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_grayscale_buffer);

        glBindTexture(GL_TEXTURE_2D, rgb_tex_id);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, CAM_WIDTH, CAM_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb_buffer);

        draw_overlay(w, h, 0); // 0: 右上角，深度图
        draw_overlay(w, h, 1); // 1: 右下角，彩色图

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

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