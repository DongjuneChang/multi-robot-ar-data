/**
 * Settings Panel
 * 부모 프레임(robot_control)에서 선택된 로봇 정보를 받아서 설정 표시
 */

// State
let currentConfig = null;
let selectedRobot = null;
let foxgloveConnected = false;

// ==================== Initialization ====================

document.addEventListener('DOMContentLoaded', async () => {
    console.log('[Settings] Initializing...');

    // 서버 정보 표시
    updateServerInfo();

    // Robot & Server 선택 로드
    await loadRobotServerOptions();

    // 부모 프레임에서 로봇 정보 가져오기
    await loadRobotFromParent();

    // 슬라이더 이벤트 초기화
    initSliders();

    // 버튼 이벤트 초기화
    initButtons();

    // System Mode 초기화
    initModeSelection();

    // Foxglove 이벤트 초기화
    initFoxglove();

    // 주기적 업데이트
    setInterval(updateTimestamp, 1000);

    // 로봇 변경 감지 (robot_control에서 변경 시)
    setInterval(checkRobotChange, 1000);

    console.log('[Settings] Initialized');
});

// ==================== Robot & Server Selection ====================

async function loadRobotServerOptions() {
    try {
        // 로봇 목록 로드
        const robotsResp = await api.getRobots();
        const robotSelect = document.getElementById('robot-type');

        if (robotsResp.robots && robotsResp.robots.length > 0) {
            robotSelect.innerHTML = '';
            for (const robot of robotsResp.robots) {
                const option = document.createElement('option');
                option.value = robot;
                option.textContent = robot;
                robotSelect.appendChild(option);
            }
        }

        // 서버 목록 로드 (network_config.yaml에서)
        const networkConfig = await api.getNetworkConfig();
        const serverSelect = document.getElementById('ros-server');

        if (networkConfig) {
            const parsed = typeof networkConfig === 'string' ? jsyaml.load(networkConfig) : networkConfig;
            const servers = parsed.servers?.ros2 || {};

            serverSelect.innerHTML = '';
            for (const [name, info] of Object.entries(servers)) {
                const option = document.createElement('option');
                option.value = name;
                option.textContent = `${name} (${info.host}:${info.port})`;
                option.dataset.host = info.host;
                option.dataset.port = info.port;
                serverSelect.appendChild(option);
            }
        }

        // 이벤트 리스너
        robotSelect.addEventListener('change', onRobotChange);
        serverSelect.addEventListener('change', onServerChange);

        // 저장된 값 복원
        const storedRobot = sessionStorage.getItem('selectedRobot');
        const storedServer = sessionStorage.getItem('selectedServer');

        if (storedRobot) robotSelect.value = storedRobot;
        if (storedServer) serverSelect.value = storedServer;

        // 서버 정보 업데이트
        onServerChange();

    } catch (e) {
        console.error('[Settings] Failed to load robot/server options:', e);
    }
}

async function onRobotChange() {
    const robotSelect = document.getElementById('robot-type');
    const robot = robotSelect.value;

    if (robot) {
        sessionStorage.setItem('selectedRobot', robot);
        selectedRobot = robot;
        await loadRobotConfig(robot);
        log(`[Settings] Robot changed to: ${robot}`);
    }
}

function onServerChange() {
    const serverSelect = document.getElementById('ros-server');
    const selectedOption = serverSelect.selectedOptions[0];

    if (selectedOption) {
        const host = selectedOption.dataset.host || '-';
        const port = selectedOption.dataset.port || '10000';

        document.getElementById('connection-ip').textContent = host;
        document.getElementById('connection-port').textContent = port;
        document.getElementById('foxglove-url').textContent = `ws://${host}:8765`;

        sessionStorage.setItem('selectedServer', serverSelect.value);
        sessionStorage.setItem('rosServerHost', host);
        sessionStorage.setItem('rosServerPort', port);

        log(`[Settings] Server changed to: ${serverSelect.value} (${host}:${port})`);
    }
}

function getSelectedMode() {
    const modeRadios = document.querySelectorAll('input[name="mode"]');
    for (const radio of modeRadios) {
        if (radio.checked) return radio.value;
    }
    return 'fake';
}

/**
 * Initialize mode radio buttons to save to sessionStorage
 */
function initModeSelection() {
    const modeRadios = document.querySelectorAll('input[name="mode"]');

    // Restore saved mode
    const savedMode = sessionStorage.getItem('systemMode') || 'fake';
    modeRadios.forEach(radio => {
        if (radio.value === savedMode) radio.checked = true;
    });

    // Save on change
    modeRadios.forEach(radio => {
        radio.addEventListener('change', () => {
            sessionStorage.setItem('systemMode', radio.value);
            log(`[Settings] System mode changed to: ${radio.value}`);
        });
    });
}

/**
 * robot_control에서 로봇이 바뀌었는지 체크
 */
async function checkRobotChange() {
    const storedRobot = sessionStorage.getItem('selectedRobot');
    if (storedRobot && storedRobot !== selectedRobot) {
        console.log(`[Settings] Robot changed: ${selectedRobot} -> ${storedRobot}`);
        selectedRobot = storedRobot;
        await loadRobotConfig(selectedRobot);
    }
}

// ==================== Server & Connection Info ====================

function updateServerInfo() {
    const host = window.location.hostname || 'localhost';
    const port = window.location.port || '5000';

    // Update connection info (use existing HTML element IDs)
    const ipEl = document.getElementById('connection-ip');
    const portEl = document.getElementById('connection-port');
    const foxgloveEl = document.getElementById('foxglove-url');

    if (ipEl) ipEl.textContent = host;
    if (portEl) portEl.textContent = port;
    if (foxgloveEl) foxgloveEl.textContent = `ws://${host}:8765`;

    // Update status bar
    const statusEl = document.getElementById('config-status');
    if (statusEl) statusEl.textContent = 'Ready';
}

async function checkApiStatus() {
    const statusEl = document.getElementById('tcp-status');
    const dotEl = document.getElementById('tcp-status-dot');

    try {
        const response = await fetch('/api/status');
        if (response.ok) {
            if (statusEl) statusEl.textContent = 'Connected';
            if (dotEl) dotEl.className = 'status-dot connected';
        }
    } catch (e) {
        if (statusEl) statusEl.textContent = 'Disconnected';
        if (dotEl) dotEl.className = 'status-dot disconnected';
    }
}

// ==================== Load Robot Config ====================

async function loadRobotFromParent() {
    // 방법 1: sessionStorage에서 로봇 정보 가져오기
    const storedRobot = sessionStorage.getItem('selectedRobot');

    // 방법 2: 부모 iframe에서 state 가져오기
    let robotFromParent = null;
    try {
        if (window.parent && window.parent !== window) {
            const parentFrame = window.parent.document.getElementById('panel-frame');
            if (parentFrame && parentFrame.contentWindow && parentFrame.contentWindow.state) {
                robotFromParent = parentFrame.contentWindow.state.selectedRobot;
            }
        }
    } catch (e) {
        console.log('[Settings] Cannot access parent frame:', e.message);
    }

    selectedRobot = robotFromParent || storedRobot || 'lite6';  // 기본값: lite6

    console.log(`[Settings] Loading robot: ${selectedRobot}`);
    document.getElementById('config-status').textContent = `Loading ${selectedRobot}...`;

    await loadRobotConfig(selectedRobot);
}

async function loadRobotConfig(robotModel) {
    try {
        // ConfigLoader 사용해서 config 로드
        if (typeof window.ConfigLoader !== 'undefined') {
            const baseUrl = getConfigBaseUrl();
            currentConfig = await window.ConfigLoader.loadRobotConfig(robotModel, {
                baseUrl: baseUrl,
                convertToRadians: false  // 표시용이므로 degrees 유지
            });
        } else {
            // ConfigLoader 없으면 직접 로드
            currentConfig = await loadConfigDirect(robotModel);
        }

        // UI 업데이트
        updateRobotDisplay(currentConfig);
        document.getElementById('config-status').textContent = `Loaded: ${robotModel}`;
        console.log(`[Settings] Config loaded for ${robotModel}`);

    } catch (error) {
        console.error('[Settings] Failed to load config:', error);
        document.getElementById('config-status').textContent = `Error: ${error.message}`;
    }
}

function getConfigBaseUrl() {
    // settings 패널 위치: web_panel/settings/
    // config 위치: config/network_robots/
    return '../../config/network_robots';
}

async function loadConfigDirect(robotModel) {
    const baseUrl = getConfigBaseUrl();

    // _defaults.yaml 로드
    const defaultsResp = await fetch(`${baseUrl}/_defaults.yaml`);
    if (!defaultsResp.ok) {
        throw new Error(`Failed to load _defaults.yaml: ${defaultsResp.status}`);
    }
    const defaultsText = await defaultsResp.text();
    const defaults = jsyaml.load(defaultsText);

    // override 로드
    const overrideResp = await fetch(`${baseUrl}/overrides/${robotModel}.yaml`);
    if (!overrideResp.ok) {
        throw new Error(`Failed to load ${robotModel}.yaml: ${overrideResp.status}`);
    }
    const overrideText = await overrideResp.text();
    const override = jsyaml.load(overrideText);

    // Deep merge
    return deepMerge(defaults, override);
}

function deepMerge(base, override) {
    const result = JSON.parse(JSON.stringify(base));
    for (const key in override) {
        if (override.hasOwnProperty(key)) {
            if (result[key] && typeof result[key] === 'object' && typeof override[key] === 'object'
                && !Array.isArray(result[key]) && !Array.isArray(override[key])) {
                result[key] = deepMerge(result[key], override[key]);
            } else {
                result[key] = JSON.parse(JSON.stringify(override[key]));
            }
        }
    }
    return result;
}

// ==================== Update Robot Display ====================

function updateRobotDisplay(config) {
    if (!config) return;

    // Robot Header
    document.getElementById('robot-name').textContent = config.robot?.model?.toUpperCase() || '-';
    document.getElementById('robot-manufacturer').textContent = config.robot?.manufacturer || '-';

    // DOF
    document.getElementById('robot-dof').textContent = config.robot?.dof || '-';

    // Joint Table
    updateJointTable(config);

    // Frame Hierarchy
    updateFrameDisplay(config);

    // Named Frames
    updateNamedFrames(config);

    // Admittance
    updateAdmittanceDisplay(config);

    // ROS2 Visualization
    updateRos2Display(config);
}

function updateJointTable(config) {
    const tbody = document.getElementById('joint-table-body');

    if (!config.joints?.names || config.joints.names.length === 0) {
        tbody.innerHTML = '<tr><td colspan="5">No joint data</td></tr>';
        return;
    }

    const names = config.joints.names;
    const zero = config.joints.zero || [];
    const home = config.joints.home || [];
    const limits = config.joints.limits || {};

    let html = '';
    for (let i = 0; i < names.length; i++) {
        const name = names[i];
        const limit = limits[name] || [-360, 360];

        html += `<tr>
            <td>${name}</td>
            <td>${zero[i] !== undefined ? zero[i] : '-'}</td>
            <td>${home[i] !== undefined ? home[i] : '-'}</td>
            <td>${limit[0]}</td>
            <td>${limit[1]}</td>
        </tr>`;
    }

    tbody.innerHTML = html;
}

function updateFrameDisplay(config) {
    const frameHierarchy = config?.frame_hierarchy;
    if (!frameHierarchy || typeof frameHierarchy !== 'object') {
        document.getElementById('base-frame').textContent = '-';
        document.getElementById('eef-frame').textContent = '-';
        document.getElementById('frame-list').innerHTML = 'No frame data';
        return;
    }

    // Network frames (base, eef)
    const networkFrames = frameHierarchy.network_frames || {};
    let baseFrame = '-';
    let eefFrame = '-';

    for (const [key, value] of Object.entries(networkFrames)) {
        if (key.includes('base')) {
            baseFrame = value.ros_frame || '-';
        } else if (key.includes('eef')) {
            eefFrame = value.ros_frame || '-';
        }
    }

    document.getElementById('base-frame').textContent = baseFrame;
    document.getElementById('eef-frame').textContent = eefFrame;

    // Local frames - 배열 확인
    const localFramesRaw = frameHierarchy.local_frames;
    const localFrames = Array.isArray(localFramesRaw) ? localFramesRaw : [];
    let frameHtml = '';

    // Network frames first
    for (const key of Object.keys(networkFrames)) {
        frameHtml += `<span class="frame-tag network">${key}</span>`;
    }

    // Local frames
    for (const frame of localFrames) {
        frameHtml += `<span class="frame-tag">${frame}</span>`;
    }

    document.getElementById('frame-list').innerHTML = frameHtml || 'No frames';
}

function updateNamedFrames(config) {
    const namedFrames = config.named_frames?.frames || {};

    // Robot Origin
    const robotOrigin = namedFrames.robot_origin?.position;
    if (robotOrigin && Array.isArray(robotOrigin)) {
        document.getElementById('robot-origin').textContent =
            `[${robotOrigin.map(v => v.toFixed(3)).join(', ')}]`;
    } else {
        document.getElementById('robot-origin').textContent = '-';
    }

    // WorkFrame
    const workFrame = namedFrames.WorkFrame?.position;
    if (workFrame && Array.isArray(workFrame)) {
        document.getElementById('workframe').textContent =
            `[${workFrame.map(v => v.toFixed(3)).join(', ')}]`;
    } else {
        document.getElementById('workframe').textContent = '-';
    }
}

function updateAdmittanceDisplay(config) {
    const admittance = config.admittance;
    if (!admittance) return;

    // Enabled - admittance는 항상 enabled (dynamics 기반)
    document.getElementById('admittance-enabled').checked = true;

    // Inertia (이전의 Mass)
    const inertia = admittance.dynamics?.inertia || [10.0, 10.0, 10.0];
    document.getElementById('mass-x').value = inertia[0];
    document.getElementById('mass-y').value = inertia[1];
    document.getElementById('mass-z').value = inertia[2];

    // Damping
    const damping = admittance.dynamics?.damping || [3.0, 3.0, 3.0];
    document.getElementById('damping-x').value = damping[0];
    document.getElementById('damping-y').value = damping[1];
    document.getElementById('damping-z').value = damping[2];

    // Stiffness (stiffness_p 사용)
    const stiffness = admittance.dynamics?.stiffness_p || [0.0, 0.0, 0.0];
    document.getElementById('stiffness-x').value = stiffness[0];
    document.getElementById('stiffness-y').value = stiffness[1];
    document.getElementById('stiffness-z').value = stiffness[2];
}

function updateRos2Display(config) {
    const ros2Viz = config.ros2_visualization;
    if (ros2Viz) {
        document.getElementById('ros2-reference-frame').textContent =
            ros2Viz.reference_frame || '-';
    }

    const subscribeFrames = config.ros2?.subscribe_frames || [];
    document.getElementById('subscribe-frame-count').textContent =
        `${subscribeFrames.length} frames`;
}

// ==================== Sliders ====================

function initSliders() {
    // Pattern Size
    const sizeSlider = document.getElementById('pattern-size');
    const sizeValue = document.getElementById('pattern-size-value');
    sizeSlider.addEventListener('input', () => {
        sizeValue.textContent = parseFloat(sizeSlider.value).toFixed(2);
    });

    // Pattern Height
    const heightSlider = document.getElementById('pattern-height');
    const heightValue = document.getElementById('pattern-height-value');
    heightSlider.addEventListener('input', () => {
        heightValue.textContent = parseFloat(heightSlider.value).toFixed(2);
    });

    // Pattern Duration
    const durationSlider = document.getElementById('pattern-duration');
    const durationValue = document.getElementById('pattern-duration-value');
    durationSlider.addEventListener('input', () => {
        durationValue.textContent = parseFloat(durationSlider.value).toFixed(1);
    });
}

// ==================== Buttons ====================

function initButtons() {
    document.getElementById('btn-apply').addEventListener('click', applySettings);
    document.getElementById('btn-reset').addEventListener('click', resetSettings);

    // ROS2 Services
    document.getElementById('btn-tcp-endpoint').addEventListener('click', toggleTcpEndpoint);
    document.getElementById('btn-device-api').addEventListener('click', toggleDeviceApi);
    document.getElementById('btn-scan-devices').addEventListener('click', scanDevices);
}

// ==================== ROS2 Services ====================

/**
 * Get the selected ROS2 server ID from dropdown
 */
function getSelectedServer() {
    const serverSelect = document.getElementById('ros-server');
    return serverSelect?.value || null;
}

/**
 * TCP Endpoint는 Unity 연결용이며 ROS2 Docker에서 실행됨
 * Web panel에서는 ROS2 서버에 요청만 전달
 * Foxglove Bridge를 통해 ROS2 연결 상태 확인 가능
 */
async function toggleTcpEndpoint() {
    const btn = document.getElementById('btn-tcp-endpoint');
    const isRunning = btn.dataset.running === 'true';
    const action = isRunning ? 'stop' : 'start';
    const server = getSelectedServer();

    // ROS2 서버 IP 가져오기
    const serverSelect = document.getElementById('ros-server');
    const selectedOption = serverSelect?.selectedOptions[0];
    const host = selectedOption?.dataset.host || 'unknown';
    const port = selectedOption?.dataset.port || '10000';

    try {
        log(`[Settings] ${action === 'start' ? 'Starting' : 'Stopping'} TCP Endpoint on ${host}:${port}...`);
        const result = await api.toggleTcpEndpoint(action, server);

        if (result.status === 'started' || result.status === 'success') {
            btn.dataset.running = 'true';
            btn.textContent = 'Stop TCP Endpoint';
            btn.classList.remove('btn-primary');
            btn.classList.add('btn-danger');
            updateTcpStatus('Running', true);
            log(`[Settings] TCP Endpoint started on ${host}:${port} (Unity 연결용)`);
        } else if (result.status === 'stopped') {
            btn.dataset.running = 'false';
            btn.textContent = 'TCP Endpoint';
            btn.classList.remove('btn-danger');
            btn.classList.add('btn-primary');
            updateTcpStatus('Stopped', false);
            log('[Settings] TCP Endpoint stopped');
        } else if (result.status === 'error') {
            log(`[Settings] TCP Endpoint error: ${result.error || 'ROS2 서버 연결 실패'} - ROS2 Docker 실행 확인 필요`, 'error');
        } else {
            log(`[Settings] TCP Endpoint response: ${JSON.stringify(result)}`);
        }
    } catch (e) {
        log(`[Settings] TCP Endpoint error: ${e.message} - ROS2 Docker (${host}:${port}) 실행 확인 필요`, 'error');
    }
}

/**
 * Device API는 HoloLens/Quest 디바이스 스캔용
 * ROS2 Docker에서 실행됨
 */
async function toggleDeviceApi() {
    const btn = document.getElementById('btn-device-api');
    const isRunning = btn.dataset.running === 'true';
    const action = isRunning ? 'stop' : 'start';
    const server = getSelectedServer();

    // ROS2 서버 IP 가져오기
    const serverSelect = document.getElementById('ros-server');
    const selectedOption = serverSelect?.selectedOptions[0];
    const host = selectedOption?.dataset.host || 'unknown';

    try {
        log(`[Settings] ${action === 'start' ? 'Starting' : 'Stopping'} Device API on ${host}...`);
        const result = await api.toggleDeviceApi(action, server);

        if (result.status === 'started' || result.status === 'success') {
            btn.dataset.running = 'true';
            btn.textContent = 'Stop Device API';
            btn.classList.remove('btn-primary');
            btn.classList.add('btn-danger');
            log(`[Settings] Device API started on ${host} (HoloLens/Quest 스캔용)`);
        } else if (result.status === 'stopped') {
            btn.dataset.running = 'false';
            btn.textContent = 'Device API';
            btn.classList.remove('btn-danger');
            btn.classList.add('btn-primary');
            log('[Settings] Device API stopped');
        } else if (result.status === 'error') {
            log(`[Settings] Device API error: ${result.error || 'ROS2 서버 연결 실패'} - ROS2 Docker 실행 확인 필요`, 'error');
        } else {
            log(`[Settings] Device API response: ${JSON.stringify(result)}`);
        }
    } catch (e) {
        log(`[Settings] Device API error: ${e.message} - ROS2 Docker (${host}) 실행 확인 필요`, 'error');
    }
}

async function scanDevices() {
    const btn = document.getElementById('btn-scan-devices');
    const deviceList = document.getElementById('device-list');
    const server = getSelectedServer();

    // ROS2 서버 IP 가져오기
    const serverSelect = document.getElementById('ros-server');
    const selectedOption = serverSelect?.selectedOptions[0];
    const host = selectedOption?.dataset.host || 'unknown';

    btn.disabled = true;
    btn.textContent = 'Scanning...';
    deviceList.innerHTML = '<p class="scanning">Scanning for devices...</p>';

    try {
        log(`[Settings] Scanning for AR devices via ${host}...`);
        const result = await api.scanDevices(server);

        if (result.devices && result.devices.length > 0) {
            let html = '';
            for (const device of result.devices) {
                html += `<div class="device-item">
                    <span class="device-name">${device.name || device.id}</span>
                    <span class="device-ip">${device.ip || '-'}</span>
                </div>`;
            }
            deviceList.innerHTML = html;
            log(`[Settings] Found ${result.devices.length} device(s)`);
        } else {
            deviceList.innerHTML = '<p class="no-devices">No devices found</p>';
            log('[Settings] No devices found');
        }
    } catch (e) {
        deviceList.innerHTML = '<p class="no-devices">Scan failed - ROS2 Docker 필요</p>';
        log(`[Settings] Device scan error: ${e.message} - ROS2 Docker (${host}) 실행 확인 필요`, 'error');
    } finally {
        btn.disabled = false;
        btn.textContent = 'Scan Devices';
    }
}

function updateTcpStatus(text, connected) {
    document.getElementById('tcp-status').textContent = text;
    document.getElementById('tcp-status-dot').className =
        connected ? 'status-dot connected' : 'status-dot disconnected';
}

function log(message, level = 'info') {
    const prefix = level === 'error' ? '[ERR]' : '[INFO]';
    console.log(`${prefix} ${message}`);
}

async function applySettings() {
    const settings = {
        pattern: {
            size: parseFloat(document.getElementById('pattern-size').value),
            height: parseFloat(document.getElementById('pattern-height').value),
            duration: parseFloat(document.getElementById('pattern-duration').value)
        },
        admittance: {
            enabled: document.getElementById('admittance-enabled').checked,
            dynamics: {
                inertia: [
                    parseFloat(document.getElementById('mass-x').value),
                    parseFloat(document.getElementById('mass-y').value),
                    parseFloat(document.getElementById('mass-z').value)
                ],
                damping: [
                    parseFloat(document.getElementById('damping-x').value),
                    parseFloat(document.getElementById('damping-y').value),
                    parseFloat(document.getElementById('damping-z').value)
                ],
                stiffness_p: [
                    parseFloat(document.getElementById('stiffness-x').value),
                    parseFloat(document.getElementById('stiffness-y').value),
                    parseFloat(document.getElementById('stiffness-z').value)
                ]
            }
        }
    };

    console.log('[Settings] Applying:', settings);

    // sessionStorage에 저장 (다른 패널에서 사용)
    sessionStorage.setItem('patternSettings', JSON.stringify(settings.pattern));
    sessionStorage.setItem('admittanceSettings', JSON.stringify(settings.admittance));

    // API로 전송 시도
    try {
        await fetch('/api/settings', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(settings)
        });
        document.getElementById('config-status').textContent = 'Settings applied';
    } catch (e) {
        document.getElementById('config-status').textContent = 'Saved locally (API unavailable)';
    }
}

function resetSettings() {
    if (currentConfig) {
        updateAdmittanceDisplay(currentConfig);
    }

    // 기본 패턴 값
    document.getElementById('pattern-size').value = 0.20;
    document.getElementById('pattern-size-value').textContent = '0.20';
    document.getElementById('pattern-height').value = 0.0;
    document.getElementById('pattern-height-value').textContent = '0.00';
    document.getElementById('pattern-duration').value = 2.0;
    document.getElementById('pattern-duration-value').textContent = '2.0';

    document.getElementById('config-status').textContent = 'Reset to defaults';
}

// ==================== Foxglove ====================

function initFoxglove() {
    document.getElementById('btn-foxglove-connect').addEventListener('click', connectFoxglove);
    document.getElementById('btn-foxglove-disconnect').addEventListener('click', disconnectFoxglove);
}

function connectFoxglove() {
    // 선택된 ROS2 서버 IP 사용 (localhost가 아님!)
    const serverSelect = document.getElementById('ros-server');
    const selectedOption = serverSelect?.selectedOptions[0];
    const host = selectedOption?.dataset.host || sessionStorage.getItem('rosServerHost') || 'localhost';
    const port = 8765;

    console.log(`[Settings] Connecting to Foxglove at ${host}:${port}`);

    if (typeof foxglove === 'undefined') {
        console.error('[Settings] Foxglove module not loaded');
        log('[Settings] Foxglove module not loaded', 'error');
        return;
    }

    // 이미 연결된 경우 연결 해제 후 재연결
    if (foxglove.connected) {
        foxglove.disconnect();
    }

    foxglove.onConnect = () => {
        foxgloveConnected = true;
        document.getElementById('foxglove-status').textContent = 'Connected';
        document.getElementById('foxglove-status-dot').className = 'status-dot connected';
        document.getElementById('btn-foxglove-connect').disabled = true;
        document.getElementById('btn-foxglove-disconnect').disabled = false;
        log(`[Settings] Foxglove connected to ${host}:${port}`);

        // 토픽 목록 표시
        setTimeout(updateFoxgloveTopics, 500);
    };

    foxglove.onDisconnect = () => {
        foxgloveConnected = false;
        document.getElementById('foxglove-status').textContent = 'Disconnected';
        document.getElementById('foxglove-status-dot').className = 'status-dot disconnected';
        document.getElementById('btn-foxglove-connect').disabled = false;
        document.getElementById('btn-foxglove-disconnect').disabled = true;
        document.getElementById('foxglove-topics').innerHTML = '<div class="no-data">Not connected</div>';
    };

    foxglove.onError = (error) => {
        console.error('[Settings] Foxglove error:', error);
        document.getElementById('foxglove-status').textContent = 'Error';
        document.getElementById('foxglove-status-dot').className = 'status-dot warning';
        log(`[Settings] Foxglove connection failed: ${host}:${port} - ROS2 Docker에서 Foxglove Bridge 실행 필요`, 'error');
    };

    foxglove.connect(host, port);
}

function disconnectFoxglove() {
    if (typeof foxglove !== 'undefined') {
        foxglove.disconnect();
    }
}

function updateFoxgloveTopics() {
    if (typeof foxglove === 'undefined') return;

    const topics = foxglove.getAvailableTopics ? foxglove.getAvailableTopics() : [];
    const container = document.getElementById('foxglove-topics');

    if (topics.length === 0) {
        container.innerHTML = '<div class="no-data">No topics available</div>';
        return;
    }

    let html = '';
    for (const topic of topics.slice(0, 20)) {  // 최대 20개 표시
        html += `<div class="topic-item">
            <span class="topic-name">${topic.topic}</span>
            <span class="topic-type">${topic.schemaName || topic.type || '-'}</span>
        </div>`;
    }

    if (topics.length > 20) {
        html += `<div class="topic-item">... and ${topics.length - 20} more</div>`;
    }

    container.innerHTML = html;
}

// ==================== Utilities ====================

function updateTimestamp() {
    document.getElementById('last-update').textContent = new Date().toLocaleTimeString();
}
