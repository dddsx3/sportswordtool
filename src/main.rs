//! NekoSportsWorldTool：无参数启动 GUI，带子命令进入 CLI（青龙等定时任务用）。
//! full 构建（默认）含 GUI + CLI；`--no-default-features` 仅 CLI。

use nekosportsworldtool::cli;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() {
        #[cfg(feature = "gui")]
        run_gui();
        #[cfg(not(feature = "gui"))]
        {
            eprintln!("当前为 CLI 构建（无 GUI）。");
            eprintln!("用法：nekosportsworldtool <子命令> [参数]");
            eprintln!("查看全部命令：nekosportsworldtool help");
            // 双击 exe 时 Windows 会新建一个只属于本进程的控制台，
            // 打印完立即退出会表现为“闪退”；此时等待回车，方便阅读提示。
            if console_owned_by_this_process() {
                eprintln!("\n按回车键退出…");
                let mut line = String::new();
                let _ = std::io::stdin().read_line(&mut line);
            }
            std::process::exit(1);
        }
    } else {
        let code = cli::main(args);
        std::process::exit(code);
    }
}

/// 控制台是否只挂着当前进程（即由双击新建，而非从已有终端启动）。
#[cfg(all(windows, not(feature = "gui")))]
fn console_owned_by_this_process() -> bool {
    #[link(name = "kernel32")]
    extern "system" {
        fn GetConsoleProcessList(lpdwProcessList: *mut u32, dwProcessCount: u32) -> u32;
    }
    let mut pids = [0u32; 2];
    let count = unsafe { GetConsoleProcessList(pids.as_mut_ptr(), pids.len() as u32) };
    count == 1
}

#[cfg(all(not(windows), not(feature = "gui")))]
fn console_owned_by_this_process() -> bool {
    false
}

#[cfg(all(windows, feature = "gui"))]
fn free_console() {
    #[link(name = "kernel32")]
    extern "system" {
        fn FreeConsole() -> i32;
    }
    unsafe {
        FreeConsole();
    }
}

#[cfg(all(not(windows), feature = "gui"))]
fn free_console() {}

#[cfg(feature = "gui")]
fn run_gui() {
    free_console();
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([880.0, 720.0])
            .with_title("NekoSportsWorldTool"),
        ..Default::default()
    };
    let result = nekosportsworldtool::run_gui(options);
    if let Err(e) = result {
        eprintln!("GUI 启动失败: {e}");
        std::process::exit(1);
    }
}
