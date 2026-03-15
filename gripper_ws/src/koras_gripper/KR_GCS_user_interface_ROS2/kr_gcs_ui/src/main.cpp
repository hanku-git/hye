/**
 * @file main.cpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief Qt based gui.
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "main_window.hpp"
#include <QApplication>
#include <QFontDatabase>
#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
    // 명령행 인자 파싱
    bool show_ui = true;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--no-ui" || arg == "--headless") {
            show_ui = false;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "사용법: " << argv[0] << " [옵션]\n";
            std::cout << "옵션:\n";
            std::cout << "  --no-ui, --headless    UI 없이 실행 (그리퍼 제어만)\n";
            std::cout << "  --help, -h             이 도움말 표시\n";
            return 0;
        }
    }

    QApplication app(argc, argv);

    /* ----------------------------------------------------------------
     * 1) 폰트 로드 (UI 모드에서만)
     * -------------------------------------------------------------- */
    if (show_ui) {
        const QString font_path = ":/font/NotoSansKR-VariableFont_wght.ttf";
        if (QFontDatabase::addApplicationFont(font_path) == -1) {
            COUT("Font not found: \"" + font_path.toStdString() + "\"");
        }
    }

    /* ----------------------------------------------------------------
     * 2) MainWindow 생성
     *    - 생성자 내부에서 flag_init_success 가 true 로 바뀌면
     *      UI 초기화가 정상적으로 끝난 것
     * -------------------------------------------------------------- */
    bool flag_init_success = false;
    gripper_ui::MainWindow w(argc, argv, flag_init_success);

    if (!flag_init_success) {
        return -1;     // 필수 리소스 또는 초기화 실패
    }

    /* ----------------------------------------------------------------
     * 3) 자동 Modbus 연결
     *    - UI 가 모두 그려진 뒤 첫 이벤트 루프 턴에서 실행
     * -------------------------------------------------------------- */
    constexpr const char *kDefaultPort  = "/dev/ttyUSB0";
    constexpr uint16_t    kDefaultSlave = 1;
    constexpr int         kDefaultBaud  = 115200;

    QTimer::singleShot(0, [&w]() {
        w.autoStartModbus(kDefaultPort, kDefaultSlave, kDefaultBaud);
    });

    /* ----------------------------------------------------------------
     * 4) 실행 모드에 따른 분기
     * -------------------------------------------------------------- */
    if (show_ui) {
        /* GUI 모드 */
        w.show();
        QObject::connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
        return app.exec();
    } else {
        /* Headless 모드 - UI 없이 그리퍼 제어만 */
        std::cout << "Headless 모드로 실행 중... (UI 없이 그리퍼 제어만)\n";
        std::cout << "그리퍼가 연결되었습니다. Ctrl+C로 종료하세요.\n";
        
        // UI를 숨기고 이벤트 루프만 실행
        w.hide();
        
        // 시그널 핸들러 설정 (Ctrl+C 감지)
        QObject::connect(&app, SIGNAL(aboutToQuit()), &app, SLOT(quit()));
        
        return app.exec();
    }
}
