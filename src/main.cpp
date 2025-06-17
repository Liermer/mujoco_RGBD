#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <unordered_map>
#include <string>
#include <cmath>
#include <chrono>    

// ---------- 全局 ----------
mjModel* m = nullptr;
mjData*  d = nullptr;
mjvScene scn;  mjvCamera cam;  mjvOption opt;  mjrContext con;

const float MOUSE_SENS  = 0.02f;
const float SCROLL_SENS = 0.2f;

// ---------- 交互用全局 ----------
bool btn_l = false, btn_m = false, btn_r = false;
double lastx = 0, lasty = 0;

// ---------- 用户输入状态（新增） ----------
int  user_mode = 1;   // 1:平地行走  2:双支撑站立  3:上楼梯  4:下楼梯  5:上斜坡  6:下斜坡
bool walk_cmd  = false; // 上方向键行走指令

// ---------- 工具函数 ----------
void zero_all_pos_actuators(){
    int na = m->nu;
    for(int i=0;i<na;i++){
        d->ctrl[i] = 0.0;
    }
}
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
// ---------- 按键操作 ----------
void key_cb(GLFWwindow*, int key, int /*scancode*/, int action, int /*mods*/){
    if(action == GLFW_PRESS){
        if(key == GLFW_KEY_R){
            mj_resetData(m, d);
            std::cout << "[INPUT] Reset simulation\n";
        }
        else if(key == GLFW_KEY_Z){
            zero_all_pos_actuators();
            std::cout << "[INPUT] Zero all actuator targets\n";
        }
        else if(key == GLFW_KEY_1){
            user_mode = 1;
            std::cout << "[INPUT] Mode set to 平地行走 (1)\n";
        }
        else if(key == GLFW_KEY_2){
            user_mode = 2;
            std::cout << "[INPUT] Mode set to 双支撑站立 (2)\n";
        }
        else if(key == GLFW_KEY_3){
            user_mode = 3;
            std::cout << "[INPUT] Mode set to 上楼梯   (3)\n";
        }
        else if(key == GLFW_KEY_4){
            user_mode = 4;
            std::cout << "[INPUT] Mode set to 下楼梯   (4)\n";
        }
        else if(key == GLFW_KEY_5){
            user_mode = 5;
            std::cout << "[INPUT] Mode set to 上斜坡   (5)\n";
        }
        else if(key == GLFW_KEY_6){
            user_mode = 6;
            std::cout << "[INPUT] Mode set to 下斜坡   (6)\n";
        }
        else if(key == GLFW_KEY_UP){
            walk_cmd = true;
            std::cout << "[INPUT] Directional command: UP pressed\n";
        }
    }
    else if(action == GLFW_RELEASE){
        if(key == GLFW_KEY_UP){
            walk_cmd = false;
            std::cout << "[INPUT] Directional command: UP released\n";
        }
    }
}

int main(){
    const char* xml = "/home/ziyi/workspace/mujoco_RGBD/model/scene/scene.xml"; //这里的路径需要根据自己的路径进行修改
    char err[1000];
    m = mj_loadXML(xml, nullptr, err, 1000);
    if(!m){
        std::cerr << "XML load error: " << err << "\n";
        return 1;
    }
    d = mj_makeData(m);

    // ---------- 建窗 & 注册回调 ----------
    glfwInit();
    GLFWwindow* win = glfwCreateWindow(1200, 900, "MuJoCo Exo", nullptr, nullptr);
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
    mjv_makeScene(m, &scn, 1000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 初始化控制目标置零
    zero_all_pos_actuators();
    set_joint_deg("pos_EL_hip_joint",  90.0);
    set_joint_deg("pos_EL_knee_joint", -90.0);

    // ------------- 同步模式主循环 -------------
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();

    while(!glfwWindowShouldClose(win)){
        // 计算“现实时间”已经过去多少秒
        auto now = clock::now();
        double realtime = std::chrono::duration<double>(now - t0).count();

        // 仿真补齐到现实时间：每个 step 对应 1/m->opt.timestep 秒
        // d->time 单位：秒
        while(d->time < realtime){
            mj_step(m, d);
        }

        // 渲染
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        int w, h;
        glfwGetFramebufferSize(win, &w, &h);
        mjrRect vp{0, 0, w, h};
        mjr_render(vp, &scn, &con);

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    // 清理
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwTerminate();
    return 0;
}
