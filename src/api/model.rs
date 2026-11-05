//! 数据模型与本地持久化。
//!
//! 持久化文件位于桌面 exe 目录或 Android 应用私有目录：identity.json（设备身份，device_id 固定复用
//! ——严禁每次随机，会触发 10121 风控）、session.json（登录态）、config.json
//! （账号/参数）、points_cache.json（点位缓存）。

pub use crate::crypto::header::HeaderIdentity;

pub const HOST: &str = "https://run.gxapp.iydsj.com";
/// 排行榜 / 违规名单域名（信封链与 RUN 相同）。
pub const DISCOVERY: &str = "https://discovery.gxapp.iydsj.com";

/// 登录态（session.json）。
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct Session {
    #[serde(default)]
    pub uid: i64,
    #[serde(default)]
    pub token: String,
    #[serde(default)]
    pub unid: String,
    #[serde(default)]
    pub name: String,
    #[serde(default = "default_weight")]
    pub weight: f64,
    #[serde(default)]
    pub username: String,
    #[serde(default)]
    pub device_id: String,
    /// 登录响应完整业务数据（我的页展示）
    #[serde(default)]
    pub profile: serde_json::Value,
}

fn default_weight() -> f64 {
    68.0
}

impl Session {
    pub fn is_logged_in(&self) -> bool {
        self.uid >= 1 && !self.token.is_empty()
    }
}

/// UI 参数（config.json）。
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Config {
    #[serde(default)]
    pub school_campus: Option<super::campus::Selection>,
    #[serde(default)]
    pub username: String,
    #[serde(default)]
    pub password: String,
    #[serde(default)]
    pub remember: bool,
    #[serde(default = "default_f32")]
    pub dist_min: f32,
    #[serde(default = "default_f32")]
    pub dist_max: f32,
    #[serde(default = "default_pace_min")]
    pub pace_min: f32,
    #[serde(default = "default_pace_max")]
    pub pace_max: f32,
    #[serde(default)]
    pub face_check: bool,
    /// 跑步时覆盖轨迹点 bdA；为空表示使用生成器的海拔曲线。
    #[serde(default)]
    pub manual_altitude: Option<f64>,
    /// 跑步时将生成器海拔曲线映射到此范围；为空表示不使用范围覆盖。
    #[serde(default)]
    pub manual_altitude_range: Option<crate::track::altitude::AltitudeRange>,
    #[serde(default = "default_ai_minutes")]
    pub ai_minutes: i64,
    #[serde(default = "default_ai_reps")]
    pub ai_reps: i64,
    /// 启动时检查更新：silent（静默，发现新版才弹窗）/ ask（每次询问）/ off
    #[serde(default = "default_update_check")]
    pub update_check: String,
    /// OSM 路网文件路径（真实道路路由用）。
    #[serde(default)]
    pub osm_path: String,
    /// 路线算法：legacy | road。
    #[serde(default = "default_route_mode")]
    pub route_mode: String,
}

fn default_route_mode() -> String {
    "legacy".into()
}

fn default_f32() -> f32 {
    1.0
}
fn default_pace_min() -> f32 {
    360.0
}
fn default_pace_max() -> f32 {
    480.0
}
fn default_ai_minutes() -> i64 {
    1
}
fn default_ai_reps() -> i64 {
    5
}
fn default_update_check() -> String {
    "silent".into()
}

impl Default for Config {
    fn default() -> Self {
        Self {
            school_campus: None,
            username: String::new(),
            password: String::new(),
            remember: false,
            dist_min: 1.0,
            dist_max: 1.5,
            pace_min: default_pace_min(),
            pace_max: default_pace_max(),
            face_check: true,
            manual_altitude: None,
            manual_altitude_range: None,
            ai_minutes: default_ai_minutes(),
            ai_reps: default_ai_reps(),
            update_check: default_update_check(),
            osm_path: String::new(),
            route_mode: default_route_mode(),
        }
    }
}

// ── 持久化（平台数据目录）──────────────────────────────────────

fn exe_dir() -> std::path::PathBuf {
    crate::platform::data_dir()
}

/// 跨进程数据锁。`File::try_lock` 与 submission_guard 使用同一套 OS 文件锁语义。
struct DataLock {
    _file: std::fs::File,
}

impl DataLock {
    /// 返回 None 表示数据目录不可写（例如只读安装目录）。此时仍允许读取有效数据，
    /// 但任何写入都会由实际文件操作明确失败，不会静默覆盖。
    fn acquire(name: &str) -> Result<Option<Self>, String> {
        let path = exe_dir().join(format!("{name}.lock"));
        let file = match std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(&path)
        {
            Ok(file) => file,
            Err(e) if e.kind() == std::io::ErrorKind::PermissionDenied => return Ok(None),
            Err(e) => return Err(format!("打开数据锁 {} 失败: {e}", path.display())),
        };
        file.try_lock()
            .map_err(|_| format!("{name} 正在被另一个进程写入，请稍后重试"))?;
        Ok(Some(Self { _file: file }))
    }
}

fn read_json_file<T: serde::de::DeserializeOwned>(
    path: &std::path::Path,
) -> Result<Option<T>, String> {
    match std::fs::read_to_string(path) {
        Ok(text) => {
            // Windows 编辑器可能写入 UTF-8 BOM；持久化文件应容忍它。
            let text = text.strip_prefix('\u{feff}').unwrap_or(&text);
            serde_json::from_str(text)
                .map(Some)
                .map_err(|e| format!("解析 {} 失败: {e}", path.display()))
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(format!("读取 {} 失败: {e}", path.display())),
    }
}

fn read_json<T: serde::de::DeserializeOwned>(name: &str) -> Option<T> {
    let dir = exe_dir();
    if let Ok(Some(value)) = read_json_file(&dir.join(name)) {
        return Some(value);
    }
    read_json_file(&dir.join(format!("{name}.bak")))
        .ok()
        .flatten()
}

fn quarantine_path(dir: &std::path::Path, name: &str) -> std::path::PathBuf {
    let stamp = chrono::Utc::now().timestamp_millis();
    let mut candidate = dir.join(format!("{name}.corrupt-{stamp}"));
    let mut suffix = 1;
    while candidate.exists() {
        candidate = dir.join(format!("{name}.corrupt-{stamp}-{suffix}"));
        suffix += 1;
    }
    candidate
}

fn quarantine(
    dir: &std::path::Path,
    name: &str,
    path: &std::path::Path,
) -> Result<std::path::PathBuf, String> {
    let target = quarantine_path(dir, name);
    std::fs::rename(path, &target)
        .map_err(|e| format!("保留损坏文件 {} 失败: {e}", path.display()))?;
    Ok(target)
}

/// 原子替换：临时文件 + fsync，旧文件先改名为 .bak。
/// 若旧文件不是合法 JSON，则移到 .corrupt-<时间戳>，避免覆盖仍可用的 .bak。
fn write_json_unlocked<T: serde::Serialize>(name: &str, value: &T) -> Result<(), String> {
    let dir = exe_dir();
    let path = dir.join(name);
    let tmp = dir.join(format!("{name}.tmp"));
    let json = serde_json::to_vec_pretty(value).map_err(|e| format!("序列化 {name} 失败: {e}"))?;
    {
        let mut file =
            std::fs::File::create(&tmp).map_err(|e| format!("写入 {} 失败: {e}", tmp.display()))?;
        use std::io::Write as _;
        file.write_all(&json)
            .map_err(|e| format!("写入 {} 失败: {e}", tmp.display()))?;
        file.sync_all()
            .map_err(|e| format!("同步 {} 失败: {e}", tmp.display()))?;
    }

    let mut old_path = None;
    if path.exists() {
        let backup = dir.join(format!("{name}.bak"));
        let valid_existing = std::fs::read_to_string(&path)
            .ok()
            .map(|text| {
                let text = text.strip_prefix('\u{feff}').unwrap_or(&text);
                serde_json::from_str::<serde_json::Value>(text).is_ok()
            })
            .unwrap_or(false);
        let target = if valid_existing {
            if backup.exists() {
                std::fs::remove_file(&backup).map_err(|e| {
                    let _ = std::fs::remove_file(&tmp);
                    format!("清理旧备份 {} 失败: {e}", backup.display())
                })?;
            }
            backup
        } else {
            quarantine_path(&dir, name)
        };
        std::fs::rename(&path, &target).map_err(|e| {
            let _ = std::fs::remove_file(&tmp);
            format!("备份 {} 失败: {e}", path.display())
        })?;
        old_path = Some(target);
    }

    match std::fs::rename(&tmp, &path) {
        Ok(()) => Ok(()),
        Err(e) => {
            if let Some(old) = old_path {
                let _ = std::fs::rename(&old, &path);
            }
            let _ = std::fs::remove_file(&tmp);
            Err(format!("替换 {} 失败: {e}", path.display()))
        }
    }
}

fn write_json<T: serde::Serialize>(name: &str, value: &T) -> Result<(), String> {
    let _lock = DataLock::acquire(name)?;
    write_json_unlocked(name, value)
}

fn default_identity(platform: &str) -> HeaderIdentity {
    let mut identity = HeaderIdentity::default();
    if platform == "android" {
        identity.platform = "android".into();
        // Generic fallback until the user opts in or fills the fields manually.
        identity.device_name = "Android".into();
        identity.os_version = "16".into();
    }
    identity
}

/// 补齐可变字段；device_id 不在这里生成，避免把损坏文件“修复”成新设备。
fn fill_identity(id: &mut HeaderIdentity) -> bool {
    let mut dirty = false;
    if id.device_id.trim().is_empty() {
        id.device_id = uuid::Uuid::new_v4().to_string().to_uppercase();
        dirty = true;
    }
    if id.app_install_time <= 0 {
        id.app_install_time =
            crate::crypto::header::HeaderIdentity::fresh_install_time(&id.platform);
        dirty = true;
    }
    if id.mac_address.is_empty() {
        id.mac_address = crate::crypto::header::random_mac();
        dirty = true;
    }
    dirty
}

/// 加载设备身份。文件损坏/缺少 device_id 时先尝试 `.bak`；两者都不可用则返回错误，
/// 不再静默生成新设备（逐请求漂移会触发 10121 风控）。
pub fn load_identity() -> Result<HeaderIdentity, String> {
    load_identity_for_platform(if cfg!(target_os = "android") {
        "android"
    } else {
        "ios"
    })
}

fn load_identity_for_platform(platform: &str) -> Result<HeaderIdentity, String> {
    let _lock = DataLock::acquire("identity.json")?;
    let dir = exe_dir();
    let path = dir.join("identity.json");
    let backup = dir.join("identity.json.bak");
    let primary_exists = path.exists();
    let mut restored = false;

    let mut primary = match read_json_file::<HeaderIdentity>(&path) {
        Ok(value) => value,
        Err(parse_error) => {
            let saved = if path.exists() {
                Some(quarantine(&dir, "identity.json", &path)?)
            } else {
                None
            };
            match read_json_file::<HeaderIdentity>(&backup)? {
                Some(id) if !id.device_id.trim().is_empty() => {
                    restored = true;
                    Some(id)
                }
                _ => {
                    return Err(format!(
                        "{parse_error}{}；请恢复 identity.json.bak 或删除损坏文件后重新生成",
                        saved
                            .map(|p| format!("，原文件已保存为 {}", p.display()))
                            .unwrap_or_default()
                    ));
                }
            }
        }
    };

    let mut id = match primary.take() {
        Some(id) if !id.device_id.trim().is_empty() => id,
        Some(_) | None => {
            if primary_exists {
                let _ = quarantine(&dir, "identity.json", &path)?;
            }
            match read_json_file::<HeaderIdentity>(&backup)? {
                Some(id) if !id.device_id.trim().is_empty() => {
                    restored = true;
                    id
                }
                _ => {
                    if !primary_exists && !backup.exists() {
                        let mut fresh = default_identity(platform);
                        fill_identity(&mut fresh);
                        write_json_unlocked("identity.json", &fresh)?;
                        return Ok(fresh);
                    }
                    return Err(format!(
                        "identity.json 损坏或缺少 device_id，已保留原文件；请恢复 {} 或删除损坏文件后重新生成",
                        backup.display()
                    ));
                }
            }
        }
    };

    let filled = fill_identity(&mut id);
    if restored || filled {
        write_json_unlocked("identity.json", &id)?;
    }
    Ok(id)
}

#[cfg(any(target_os = "android", test))]
#[derive(Debug, serde::Deserialize)]
pub struct DeviceInfo {
    pub manufacturer: String,
    pub model: String,
    pub os_version: String,
}

#[cfg(any(target_os = "android", test))]
pub fn identity_with_device_info(
    identity: &HeaderIdentity,
    info: &DeviceInfo,
) -> Result<HeaderIdentity, String> {
    if info.model.trim().is_empty() || info.os_version.trim().is_empty() {
        return Err("未能读取完整的机型和系统版本，请手动填写".into());
    }
    let mut updated = identity.clone();
    updated.platform = "android".into();
    updated.device_name = info.model.trim().into();
    updated.manufacturer = info.manufacturer.trim().into();
    updated.os_version = info.os_version.trim().into();
    Ok(updated)
}

pub fn save_identity(id: &HeaderIdentity) -> Result<(), String> {
    if id.device_id.trim().is_empty() {
        return Err("设备 ID 不能为空；请先在设备信息页生成或填写".into());
    }
    let _lock = DataLock::acquire("identity.json")?;
    let dir = exe_dir();
    let path = dir.join("identity.json");
    // 已存在但 device_id 为空/结构不合法的文件不能覆盖仍可用的 .bak。
    match read_json_file::<HeaderIdentity>(&path) {
        Ok(Some(existing)) if existing.device_id.trim().is_empty() => {
            quarantine(&dir, "identity.json", &path)?;
        }
        Err(_) if path.exists() => {
            quarantine(&dir, "identity.json", &path)?;
        }
        _ => {}
    }
    write_json_unlocked("identity.json", id)
}

/// AI 项目列表永久缓存：拉取成功即落盘，网络异常时兜底展示。
pub fn load_ai_sports() -> Option<Vec<crate::api::ai::AiSport>> {
    read_json("ai_sports.json")
}

pub fn save_ai_sports(list: &[crate::api::ai::AiSport]) -> Result<(), String> {
    write_json("ai_sports.json", &list.to_vec())
}

pub fn load_session() -> Session {
    read_json("session.json").unwrap_or_default()
}

pub fn save_session(s: &Session) -> Result<(), String> {
    write_json("session.json", s)
}

pub fn clear_session() {
    if let Ok(_lock) = DataLock::acquire("session.json") {
        let _ = std::fs::remove_file(exe_dir().join("session.json"));
    }
}

pub fn load_config() -> Config {
    read_json("config.json").unwrap_or_default()
}

pub fn save_config(c: &Config) -> Result<(), String> {
    write_json("config.json", c)
}

/// 设备级恒定参数与计数器。
///
/// NEUQ 真机指纹（33 条实测）：GPS accuracy 字段 98% 恒定（该机为 3.79），
/// 点 id 与 10s 窗 id 为同一设备上跨次运行递增的计数器（点 id 中位 12031、
/// 窗 id 中位 ~5000）。首次生成时抽取一次基准并落盘，此后跨次递增。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DeviceProfile {
    pub radius_base: f64,
    pub point_id: i64,
    pub window_id: i64,
    /// 五点载荷的点位 id（真机为 App 内部点表计数器，NEUQ 6 月实测 346..350）。
    pub five_point_id: i64,
}

const DEVICE_PROFILE_FILE: &str = "device_profile.json";

pub fn device_profile() -> DeviceProfile {
    if let Some(p) = read_json::<DeviceProfile>(DEVICE_PROFILE_FILE) {
        return p;
    }
    use rand::Rng;
    let mut rng = rand::thread_rng();
    let profile = DeviceProfile {
        // 7 位小数基准：f32 最短表示与真人 3.7900925 形态一致
        radius_base: ((3.5 + rng.gen::<f64>() * 4.5) * 1e7).round() / 1e7,
        // 真人同设备 DB 历史连贯（33 条库 id 终点≈16217）：新账号从
        // 16250 起无缝衔接且不与真人区间碰撞（盲测3 指纹 15）
        point_id: 16_250 + (rng.gen::<f64>() * 600.0) as i64,
        window_id: 4_000 + (rng.gen::<f64>() * 3_000.0) as i64,
        five_point_id: 340 + (rng.gen::<f64>() * 160.0) as i64,
    };
    let _ = write_json(DEVICE_PROFILE_FILE, &profile);
    profile
}

/// 一次完整轨迹生成后递增设备计数器（点数 + 窗口数）。
pub fn bump_device_counters(points: i64, windows: i64) {
    let mut p = device_profile();
    p.point_id += points;
    p.window_id += windows;
    let _ = write_json(DEVICE_PROFILE_FILE, &p);
}

#[cfg(test)]
mod storage_tests {
    use super::*;

    #[test]
    fn android_identity_import_persists_brand_and_reuses_uuid() {
        let directory =
            std::env::temp_dir().join(format!("neko-identity-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&directory).unwrap();
        crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = Some(directory.clone()));
        struct Cleanup(std::path::PathBuf);
        impl Drop for Cleanup {
            fn drop(&mut self) {
                crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = None);
                let _ = std::fs::remove_dir_all(&self.0);
            }
        }
        let _cleanup = Cleanup(directory);
        let first = load_identity_for_platform("android").unwrap();
        assert_eq!(first.platform, "android");
        assert_ne!(first.device_name, "iPhone");
        assert!(uuid::Uuid::parse_str(&first.device_id).is_ok());
        let again = load_identity_for_platform("android").unwrap();
        assert_eq!(
            serde_json::to_value(&first).unwrap(),
            serde_json::to_value(&again).unwrap()
        );

        let mut existing = first.clone();
        existing.platform = "ios".into();
        existing.device_name = "Existing manual name".into();
        existing.idfa = "manual-value".into();
        let mut legacy = serde_json::to_value(&existing).unwrap();
        legacy.as_object_mut().unwrap().remove("manufacturer");
        write_json("identity.json", &legacy).unwrap();
        let loaded = load_identity_for_platform("android").unwrap();
        assert_eq!(
            serde_json::to_value(&loaded).unwrap(),
            serde_json::to_value(&existing).unwrap(),
            "opening an existing install must not overwrite a manual identity"
        );

        let info: DeviceInfo = serde_json::from_str(
            r#"{"manufacturer":"Example","model":"Phone 16","os_version":"16"}"#,
        )
        .unwrap();
        assert_eq!(info.manufacturer, "Example");
        let imported = identity_with_device_info(&loaded, &info).unwrap();
        assert_eq!(imported.platform, "android");
        assert_eq!(imported.device_name, "Phone 16");
        assert_eq!(imported.os_version, "16");
        let mut expected = serde_json::to_value(&existing).unwrap();
        expected["platform"] = "android".into();
        expected["device_name"] = "Phone 16".into();
        expected["manufacturer"] = "Example".into();
        expected["os_version"] = "16".into();
        assert_eq!(
            serde_json::to_value(&imported).unwrap(),
            expected,
            "import must preserve UUID, MAC, manual IMEI/IDFA, install time and location"
        );
        assert_eq!(
            load_identity().unwrap().device_name,
            existing.device_name,
            "preview must not save implicitly"
        );
        save_identity(&imported).unwrap();
        let stored: serde_json::Value = read_json("identity.json").unwrap();
        assert_eq!(
            stored.get("manufacturer").and_then(|value| value.as_str()),
            Some("Example"),
            "the consented brand must be written to identity.json"
        );
        let reloaded = load_identity().unwrap();
        assert_eq!(
            serde_json::to_value(&reloaded).unwrap(),
            serde_json::to_value(&imported).unwrap(),
            "brand and identity must survive a fresh load from disk"
        );
        assert_eq!(reloaded.device_id, first.device_id);
        let (header, _) =
            crate::crypto::header::build_header_for(&reloaded, 0, "", Some(1_700_000_000_000));
        let header: serde_json::Value = serde_json::from_str(&header).unwrap();
        assert_eq!(header["DeviceId"], first.device_id);
        assert_eq!(
            header["deviceName"], "Phone 16",
            "brand must not be prepended to the protocol model field"
        );
        assert_eq!(header["osVersion"], "16");
        assert!(
            header.get("manufacturer").is_none() && header.get("brand").is_none(),
            "persisting local brand metadata must not invent new protocol fields"
        );
    }

    #[test]
    fn incomplete_native_device_info_cannot_replace_identity() {
        let identity = HeaderIdentity::default();
        for (model, os_version) in [("", "16"), ("Phone", " ")] {
            let info = DeviceInfo {
                manufacturer: String::new(),
                model: model.into(),
                os_version: os_version.into(),
            };
            assert!(identity_with_device_info(&identity, &info).is_err());
        }
    }

    #[test]
    fn session_roundtrip_and_logout_use_private_data_directory() {
        let directory = std::env::temp_dir().join(format!("neko-storage-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&directory).unwrap();
        crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = Some(directory.clone()));
        struct Cleanup(std::path::PathBuf);
        impl Drop for Cleanup {
            fn drop(&mut self) {
                crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = None);
                let _ = std::fs::remove_dir_all(&self.0);
            }
        }
        let _cleanup = Cleanup(directory.clone());
        assert_eq!(
            exe_dir(),
            directory,
            "storage must use the injected app directory"
        );
        let session = Session {
            uid: 123,
            token: "test-only-token".into(),
            ..Default::default()
        };
        save_session(&session).unwrap();
        assert!(directory.join("session.json").is_file());
        assert_eq!(load_session().uid, 123);
        assert_eq!(load_session().token, session.token);
        clear_session();
        assert!(!directory.join("session.json").exists());
        assert_eq!(load_session().uid, 0);
    }

    fn temp_root(tag: &str) -> (std::path::PathBuf, impl Drop) {
        let directory = std::env::temp_dir().join(format!("neko-{tag}-{}", uuid::Uuid::new_v4()));
        std::fs::create_dir_all(&directory).unwrap();
        crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = Some(directory.clone()));
        struct Cleanup(std::path::PathBuf);
        impl Drop for Cleanup {
            fn drop(&mut self) {
                crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = None);
                let _ = std::fs::remove_dir_all(&self.0);
            }
        }
        (directory.clone(), Cleanup(directory))
    }

    #[test]
    fn atomic_write_keeps_previous_backup_and_recovers_corrupt_primary() {
        let (directory, _cleanup) = temp_root("atomic");
        let mut first = Config::default();
        first.username = "one".into();
        save_config(&first).unwrap();
        let mut second = first.clone();
        second.username = "two".into();
        save_config(&second).unwrap();

        let primary: Config =
            serde_json::from_str(&std::fs::read_to_string(directory.join("config.json")).unwrap())
                .unwrap();
        let backup: Config = serde_json::from_str(
            &std::fs::read_to_string(directory.join("config.json.bak")).unwrap(),
        )
        .unwrap();
        assert_eq!(primary.username, "two");
        assert_eq!(backup.username, "one");

        std::fs::write(directory.join("config.json"), b"{bad").unwrap();
        assert_eq!(load_config().username, "one");
    }

    #[test]
    fn corrupt_identity_recovers_from_backup_without_rotating() {
        let (directory, _cleanup) = temp_root("identity-recover");
        let first = load_identity_for_platform("ios").unwrap();
        save_identity(&first).unwrap();
        std::fs::write(directory.join("identity.json"), b"{bad").unwrap();

        let recovered = load_identity_for_platform("ios").unwrap();
        assert_eq!(recovered.device_id, first.device_id);
        assert!(directory.join("identity.json").is_file());
        assert!(directory.join("identity.json.bak").is_file());
    }

    #[test]
    fn corrupt_identity_without_backup_is_rejected_not_regenerated() {
        let (directory, _cleanup) = temp_root("identity-reject");
        std::fs::write(directory.join("identity.json"), b"{bad").unwrap();

        assert!(load_identity_for_platform("ios").is_err());
        assert!(!directory.join("identity.json").exists());
        let quarantined = std::fs::read_dir(&directory)
            .unwrap()
            .filter_map(|entry| entry.ok())
            .filter(|entry| {
                entry
                    .file_name()
                    .to_string_lossy()
                    .starts_with("identity.json.corrupt-")
            })
            .count();
        assert_eq!(quarantined, 1);
    }

    #[test]
    fn empty_identity_file_is_rejected_not_regenerated() {
        let (directory, _cleanup) = temp_root("identity-empty");
        std::fs::write(directory.join("identity.json"), b"{}").unwrap();

        assert!(load_identity_for_platform("ios").is_err());
        assert!(!directory.join("identity.json").exists());
    }

    #[test]
    fn save_identity_rejects_empty_device_id() {
        let (directory, _cleanup) = temp_root("identity-save-empty");
        assert!(save_identity(&HeaderIdentity::default()).is_err());
        assert!(!directory.join("identity.json").exists());
    }
}

/// 点位缓存：{ts_ms, points}，TTL 300s（服务端限流 10603：5 分钟 3 次）。
pub const POINTS_TTL_MS: i64 = 300_000;

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct PointsCacheScope {
    pub uid: i64,
    pub unid: String,
    pub run_area_id: Option<String>,
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
struct PointsCache {
    ts: i64,
    anchor: crate::location::Coordinate,
    scope: PointsCacheScope,
    points: Vec<serde_json::Value>,
}

impl PointsCache {
    fn matches(
        &self,
        anchor: crate::location::Coordinate,
        scope: &PointsCacheScope,
        now: i64,
    ) -> bool {
        self.scope == *scope
            && self.anchor.is_near(anchor, 0.0001)
            && !self.points.is_empty()
            && now
                .checked_sub(self.ts)
                .is_some_and(|age| (0..POINTS_TTL_MS).contains(&age))
    }
}

/// 旧版未记录账号、学校、区域的缓存自然失效；拒绝过期或来自未来的缓存。
pub fn load_points_cache_for(
    anchor: crate::location::Coordinate,
    scope: &PointsCacheScope,
) -> Option<(i64, Vec<serde_json::Value>)> {
    let v: serde_json::Value = read_json("points_cache.json")?;
    let cache: PointsCache = serde_json::from_value(v).ok()?;
    if !cache.matches(anchor, scope, crate::crypto::envelope::now_ms()) {
        return None;
    }
    let ts = cache.ts;
    let pts = cache.points;
    Some((ts, pts))
}

pub fn save_points_cache(
    anchor: crate::location::Coordinate,
    scope: &PointsCacheScope,
    points: &[serde_json::Value],
) -> Result<(), String> {
    let doc = PointsCache {
        ts: crate::crypto::envelope::now_ms(),
        anchor,
        scope: scope.clone(),
        points: points.to_vec(),
    };
    write_json("points_cache.json", &doc)
}

#[cfg(test)]
mod points_cache_tests {
    use super::*;
    #[test]
    fn cache_is_scoped_to_anchor() {
        let cache = PointsCache {
            ts: 1,
            anchor: crate::location::Coordinate::new(39.9, 116.4, 0.0).unwrap(),
            scope: PointsCacheScope {
                uid: 1,
                unid: "school".into(),
                run_area_id: None,
            },
            points: vec![serde_json::json!({"lat":39.9,"lon":116.4})],
        };
        assert!(cache.matches(cache.anchor, &cache.scope, 1));
        assert!(cache.matches(cache.anchor, &cache.scope, POINTS_TTL_MS));
        assert!(!cache.matches(cache.anchor, &cache.scope, POINTS_TTL_MS + 1));
        assert!(!cache.matches(cache.anchor, &cache.scope, 0));
        for scope in [
            PointsCacheScope {
                uid: 2,
                ..cache.scope.clone()
            },
            PointsCacheScope {
                unid: "other".into(),
                ..cache.scope.clone()
            },
            PointsCacheScope {
                run_area_id: Some("42".into()),
                ..cache.scope.clone()
            },
        ] {
            assert!(!cache.matches(cache.anchor, &scope, 1));
        }
        assert!(!cache.matches(
            crate::location::Coordinate::new(38.9, 121.5, 0.0).unwrap(),
            &cache.scope,
            1
        ));
        let mut old = serde_json::to_value(&cache).unwrap();
        old.as_object_mut().unwrap().remove("scope");
        assert!(serde_json::from_value::<PointsCache>(old).is_err());
        let decoded: PointsCache =
            serde_json::from_value(serde_json::to_value(cache).unwrap()).unwrap();
        assert!(decoded.anchor.is_near(
            crate::location::Coordinate::new(39.9, 116.4, 0.0).unwrap(),
            0.0001
        ));
        assert!(!decoded.anchor.is_near(
            crate::location::Coordinate::new(38.9, 121.5, 0.0).unwrap(),
            0.0001
        ));
    }
}
