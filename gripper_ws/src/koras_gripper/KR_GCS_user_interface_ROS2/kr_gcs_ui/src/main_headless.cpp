/**
 * @file main_headless.cpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief Headless mode for gripper control without UI
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "datc_comm_interface.hpp"
#include <QApplication>
#include <QTimer>
#include <iostream>
#include <string>
#include <signal.h>

// 전역 변수로 앱 포인터 저장 (시그널 핸들러에서 사용)
QApplication* g_app = nullptr;
DatcCommInterface* g_datc_interface = nullptr;

// 시그널 핸들러
void signalHandler(int signum) {
    std::cout << "\n종료 신호를 받았습니다. 그리퍼를 안전하게 종료합니다...\n";
    if (g_datc_interface) {
        g_datc_interface->modbusRelease();
    }
    if (g_app) {
        g_app->quit();
    }
    exit(signum);
}

int main(int argc, char *argv[])
{
    // 명령행 인자 파싱
    std::string port = "/dev/ttyUSB0";
    uint16_t slave = 1;
    int baud = 115200;
    bool help_requested = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            help_requested = true;
        } else if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--slave" && i + 1 < argc) {
            slave = std::stoi(argv[++i]);
        } else if (arg == "--baud" && i + 1 < argc) {
            baud = std::stoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            help_requested = true;
        }
    }

    if (help_requested) {
        std::cout << "사용법: " << argv[0] << " [옵션]\n";
        std::cout << "옵션:\n";
        std::cout << "  --port <포트>      시리얼 포트 지정 (기본값: /dev/ttyUSB0)\n";
        std::cout << "  --slave <주소>     슬레이브 주소 지정 (기본값: 1)\n";
        std::cout << "  --baud <속도>      보드레이트 지정 (기본값: 115200)\n";
        std::cout << "  --help, -h         이 도움말 표시\n";
        std::cout << "\n예시:\n";
        std::cout << "  " << argv[0] << " --port /dev/ttyUSB1 --slave 2\n";
        return 0;
    }

    QApplication app(argc, argv);
    g_app = &app;

    // 시그널 핸들러 등록
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "=== 그리퍼 Headless 모드 ===\n";
    std::cout << "포트: " << port << "\n";
    std::cout << "슬레이브 주소: " << slave << "\n";
    std::cout << "보드레이트: " << baud << "\n";
    std::cout << "========================\n\n";

    /* ----------------------------------------------------------------
     * DatcCommInterface 생성
     * -------------------------------------------------------------- */
    g_datc_interface = new DatcCommInterface(argc, argv);
    g_datc_interface->start();

    /* ----------------------------------------------------------------
     * Modbus 연결
     * -------------------------------------------------------------- */
    std::cout << "그리퍼에 연결 중...\n";
    if (g_datc_interface->init(port.c_str(), slave, baud)) {
        std::cout << "그리퍼 연결 성공!\n";
        g_datc_interface->motorEnable();
    } else {
        std::cerr << "그리퍼 연결 실패! 포트나 권한을 확인하세요.\n";
        delete g_datc_interface;
        return -1;
    }

    /* ----------------------------------------------------------------
     * Headless 모드 실행
     * -------------------------------------------------------------- */
    std::cout << "그리퍼가 준비되었습니다.\n";
    std::cout << "Ctrl+C를 눌러 종료하세요.\n\n";
    
    // 주기적으로 상태 출력
    QTimer status_timer;
    QObject::connect(&status_timer, &QTimer::timeout, [&]() {
        if (g_datc_interface) {
            DatcStatus status = g_datc_interface->getDatcStatus();
            std::cout << "\r상태: " << status.status_str 
                      << " | 위치: " << (double)status.finger_pos / 10.0 << "%"
                      << " | 전류: " << status.motor_cur << "mA    " << std::flush;
        }
    });
    status_timer.start(1000); // 1초마다 상태 출력
    
    int result = app.exec();
    
    // 정리
    if (g_datc_interface) {
        g_datc_interface->modbusRelease();
        delete g_datc_interface;
    }
    
    return result;
}
