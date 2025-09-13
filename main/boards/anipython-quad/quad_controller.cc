/*
    Quad机器人控制器 - MCP协议版本
*/

#include <cJSON.h>
#include <esp_log.h>

#include <cstring>

#include "application.h"
#include "board.h"
#include "config.h"
#include "mcp_server.h"
#include "quad_movements.h"
#include "sdkconfig.h"
#include "settings.h"

#define TAG "QuadController"

class QuadController {
private:
    Quad quad_;
    TaskHandle_t action_task_handle_ = nullptr;
    QueueHandle_t action_queue_;
    bool is_action_in_progress_ = false;

    struct QuadActionParams {
        int action_type;
        int steps;
        // int speed;
        // int direction;
        // int amount;
    };

    enum ActionType {
        ACTION_FORWALK = 1,
        ACTION_BACKWALK = 2,
        ACTION_HOME = 3
    };

    static void ActionTask(void* arg) {
        QuadController* controller = static_cast<QuadController*>(arg);
        QuadActionParams params;
        controller->quad_.AttachServos();

        while (true) {
            if (xQueueReceive(controller->action_queue_, &params, pdMS_TO_TICKS(1000)) == pdTRUE) {
                ESP_LOGI(TAG, "执行动作: %d", params.action_type);
                controller->is_action_in_progress_ = true;

                switch (params.action_type) {
                    case ACTION_FORWALK:
                        controller->quad_.Forward(8, 1000);
                        ESP_LOGI(TAG, "xQueueReceive FORWALK~~");
                        break;

                    case ACTION_BACKWALK:
                        controller->quad_.Backward(4, 1000);
                        ESP_LOGI(TAG, "xQueueReceive Backward~~");
                        break;
                    
                    case ACTION_HOME:
                        controller->quad_.Home();
                        ESP_LOGI(TAG, "xQueueReceive Home~~");
                        break;
                }
                if (params.action_type != ACTION_HOME) {
                     controller->quad_.Home();
                }
                controller->is_action_in_progress_ = false;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }

    void StartActionTaskIfNeeded() {
        if (action_task_handle_ == nullptr) {
            xTaskCreate(ActionTask, "quad_action", 1024 * 3, this, configMAX_PRIORITIES - 1,
                        &action_task_handle_);
        }
    }

    void QueueAction(int action_type, int steps) {

        ESP_LOGI(TAG, "动作控制: 类型=%d, 步数=%d", action_type, steps);

        QuadActionParams params = {action_type, steps};
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }


public:
    QuadController() {
        quad_.Init(FLU_PIN, FRU_PIN, FLD_PIN, FRD_PIN, BLU_PIN, BRU_PIN, BLD_PIN, BRD_PIN);

        action_queue_ = xQueueCreate(10, sizeof(QuadActionParams));

        QueueAction(ACTION_HOME, 1);

        RegisterMcpTools();
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        ESP_LOGI(TAG, "开始注册MCP工具...");

        // 基础移动动作
        mcp_server.AddTool("self.quad.forward",
                           "前进",
                           PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               QueueAction(ACTION_FORWALK, 2);
                               // quad_.Forward(8, 800);
                               ESP_LOGI(TAG, "QueueAction(ACTION_FORWALK, 2);");
                               return true;
                           });
        
        // mcp_server.AddTool("self.chassis.go_forward", "前进", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
        //     SendUartMessage("x0.0 y1.0");
        //     return true;
        // });
        
        mcp_server.AddTool("self.quad.backward",
                           "后退",
                           PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               QueueAction(ACTION_BACKWALK, 2);
                               ESP_LOGI(TAG, "QueueAction(ACTION_BACKWALK, 2);~~");
                               return true;
                           });

        // 系统工具
        // mcp_server.AddTool("self.quad.stop", "立即停止", PropertyList(),
        //                    [this](const PropertyList& properties) -> ReturnValue {
        //                        if (action_task_handle_ != nullptr) {
        //                            vTaskDelete(action_task_handle_);
        //                            action_task_handle_ = nullptr;
        //                        }
        //                        is_action_in_progress_ = false;
        //                        xQueueReset(action_queue_);

        //                        QueueAction(ACTION_HOME, 1, 1000, 1, 0);
        //                        return true;
        //                    });

        ESP_LOGI(TAG, "MCP工具注册完成");
    }

    ~QuadController() {
        if (action_task_handle_ != nullptr) {
            vTaskDelete(action_task_handle_);
            action_task_handle_ = nullptr;
        }
        vQueueDelete(action_queue_);
    }
};

static QuadController* g_quad_controller = nullptr;

void InitializeQuadController() {
    if (g_quad_controller == nullptr) {
        g_quad_controller = new QuadController();
        ESP_LOGI(TAG, "Quad控制器已初始化并注册MCP工具");
    }
}
