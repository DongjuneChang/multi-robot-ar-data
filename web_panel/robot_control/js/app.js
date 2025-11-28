/**
 * Multi-Robot Control Panel v4.5.3 HTML
 * Main Application Logic
 *
 * Architecture:
 * - FastAPI Server serves HTML + REST API (port 5000)
 * - Foxglove Bridge provides real-time ROS2 data (port 8765)
 * - HoloLens Edge browser accesses via http://<linux-ip>:5000
 */

// ==================== Global State ====================

const state = {
    // Connection
    serverHost: null,
    serverPort: 5000,
    foxglovePort: 8765,
    foxgloveHost: null,  // ROS2 server IP for Foxglove

    // Robot
    selectedRobot: null,
    robotConfig: null,
    robotFrames: null,
    availableRobots: [],

    // Process states
    moveitRunning: false,
    controllerRunning: false,
    patternServiceRunning: false,
    tcpEndpointRunning: false,
    testRunning: false,
    worldTfRunning: false,
    vizLaunchRunning: false,
    deviceApiRunning: false,
    foxgloveConnected: false,

    // Device config
    deviceConfig: null,
    networkConfig: null,
    ros2InterfaceConfig: null,  // ros2_interface/_defaults.yaml
    detectedDevices: [],
    rosServerIp: null,

    // Available ROS2 services (from Foxglove)
    availableServices: []
};

// ==================== Initialization ====================

document.addEventListener('DOMContentLoaded', async () => {
    log('Initializing Control Panel...', 'info');

    // Detect server from current page URL
    state.serverHost = window.location.hostname || 'localhost';
    state.serverPort = window.location.port || 5000;

    // Update UI
    document.getElementById('server-url').textContent =
        `${state.serverHost}:${state.serverPort}`;

    // Initialize components
    initEventListeners();
    initSliders();
    initFoxglove();

    // Load configuration
    try {
        await loadConfigurations();
        loadServerFromSettings();
        await loadRobotFromSettings();
        await loadRos2InterfaceConfig();
        log('Control Panel initialized successfully', 'success');
    } catch (error) {
        log(`Initialization error: ${error.message}`, 'error');
    }

    // Watch for settings changes (from Settings page)
    setInterval(checkSettingsChange, 2000);

    // Start status polling
    startStatusPolling();

    // Auto-connect to Foxglove if server is set
    if (state.rosServerIp) {
        connectFoxglove();
    }
});

/**
 * Initialize event listeners for all buttons
 */
function initEventListeners() {
    // System Control
    document.getElementById('btn-clean').addEventListener('click', cleanSystem);
    document.getElementById('btn-moveit').addEventListener('click', toggleMoveIt);
    document.getElementById('btn-controller').addEventListener('click', toggleController);
    document.getElementById('btn-world-tf').addEventListener('click', toggleWorldTf);
    document.getElementById('btn-viz-system').addEventListener('click', toggleVizSystem);
    document.getElementById('btn-start-path').addEventListener('click', startPathPlanning);
    document.getElementById('btn-stop-path').addEventListener('click', stopPathPlanning);

    // Pattern Test
    document.getElementById('btn-pattern-service').addEventListener('click', togglePatternService);
    document.getElementById('btn-preview').addEventListener('click', previewPatternTest);
    document.getElementById('btn-start-test').addEventListener('click', startPatternTest);
    document.getElementById('btn-stop-test').addEventListener('click', stopPatternTest);

    // Joint Control
    const btnHome = document.getElementById('btn-home');
    const btnZero = document.getElementById('btn-zero');
    if (btnHome) btnHome.addEventListener('click', moveToHome);
    if (btnZero) btnZero.addEventListener('click', moveToZero);

    // Output
    document.getElementById('btn-clear-output').addEventListener('click', clearOutput);
    document.getElementById('btn-copy-output').addEventListener('click', copyOutput);
}

/**
 * Initialize slider value displays
 */
function initSliders() {
    const sizeSlider = document.getElementById('size-slider');
    const sizeValue = document.getElementById('size-value');
    sizeSlider.addEventListener('input', () => {
        sizeValue.textContent = parseFloat(sizeSlider.value).toFixed(2);
    });

    const speedSlider = document.getElementById('speed-slider');
    const speedValue = document.getElementById('speed-value');
    speedSlider.addEventListener('input', () => {
        speedValue.textContent = parseFloat(speedSlider.value).toFixed(1) + 'x';
    });
}

// ==================== Configuration Loading ====================

/**
 * Get config base path (absolute path)
 */
function getConfigBasePath() {
    return '/config/';
}

/**
 * Load device and network configurations from shared_data
 */
async function loadConfigurations() {
    const basePath = getConfigBasePath();

    try {
        // Load device config - 직접 YAML 파일 fetch
        const deviceResp = await fetch(`${basePath}device_config.yaml`);
        if (deviceResp.ok) {
            const deviceYaml = await deviceResp.text();
            state.deviceConfig = jsyaml.load(deviceYaml);
            log('Loaded device_config.yaml', 'debug');
        }

        // Load network config - 직접 YAML 파일 fetch
        const networkResp = await fetch(`${basePath}network_config.yaml`);
        if (networkResp.ok) {
            const networkYaml = await networkResp.text();
            state.networkConfig = jsyaml.load(networkYaml);
            log('Loaded network_config.yaml', 'debug');
        }

        // Load map_data for user info
        const mapResp = await fetch(`${basePath}map_data.yaml`);
        if (mapResp.ok) {
            const mapYaml = await mapResp.text();
            state.mapData = jsyaml.load(mapYaml);
            log('Loaded map_data.yaml', 'debug');
        }

    } catch (error) {
        log(`Config load error: ${error.message}`, 'warn');
    }
}

/**
 * Load server info from sessionStorage (set by Settings page)
 */
function loadServerFromSettings() {
    const serverHost = sessionStorage.getItem('rosServerHost');
    const serverPort = sessionStorage.getItem('rosServerPort');
    const serverName = sessionStorage.getItem('selectedServer');

    if (serverHost) {
        state.rosServerIp = serverHost;
        document.getElementById('header-server').textContent = serverName ? `${serverName}` : serverHost;
        log(`Server: ${serverHost}:${serverPort || 10000}`, 'info');
    }
}

/**
 * Load robot from sessionStorage (set by Settings page) or default
 */
async function loadRobotFromSettings() {
    const storedRobot = sessionStorage.getItem('selectedRobot');

    if (storedRobot) {
        state.selectedRobot = storedRobot;
        document.getElementById('header-robot').textContent = storedRobot.toUpperCase();
        log(`Robot: ${storedRobot} (from Settings)`, 'info');

        // Load robot config
        try {
            const config = await api.getRobotConfig(storedRobot);
            state.robotConfig = config;
            state.robotFrames = config.named_frames || config.frame_hierarchy || {};
        } catch (error) {
            log(`Failed to load robot config: ${error.message}`, 'warn');
        }
    } else {
        // Try to load from map_data.yaml and set default
        await loadDefaultRobot();
    }
}

/**
 * Load ros2_interface config (services, topics, commands from YAML)
 * Uses Ros2InterfaceLoader to merge _defaults.yaml + overrides/{robot}.yaml
 */
async function loadRos2InterfaceConfig() {
    const robotModel = state.selectedRobot;
    if (!robotModel) {
        log('No robot selected, cannot load ros2_interface config', 'warn');
        return;
    }

    const basePath = getConfigBasePath();
    try {
        // Ros2InterfaceLoader.load() handles defaults + override merge
        state.ros2InterfaceConfig = await Ros2InterfaceLoader.load(
            robotModel,
            { baseUrl: `${basePath}ros2_interface` }
        );
        log(`Loaded ros2_interface config for ${robotModel}`, 'debug');

        // Log available services
        if (state.ros2InterfaceConfig?.services) {
            const serviceNames = Object.keys(state.ros2InterfaceConfig.services);
            log(`Services: ${serviceNames.join(', ')}`, 'debug');
        }
    } catch (error) {
        log(`Failed to load ros2_interface config: ${error.message}`, 'warn');
    }
}

/**
 * Get service name from ros2_interface config
 * @param {string} key - Service key (e.g., 'pattern_start')
 * @returns {string|null} Service name or null
 */
function getServiceName(key) {
    const name = Ros2InterfaceLoader.getServiceName(state.ros2InterfaceConfig, key);
    if (!name) {
        log(`Service '${key}' not found in ros2_interface config`, 'warn');
    }
    return name;
}

/**
 * Get topic name from ros2_interface config
 * @param {string} key - Topic key (e.g., 'joint_states')
 * @returns {string|null} Topic name or null
 */
function getTopicName(key) {
    const name = Ros2InterfaceLoader.getTopicName(state.ros2InterfaceConfig, key);
    if (!name) {
        log(`Topic '${key}' not found in ros2_interface config`, 'warn');
    }
    return name;
}

/**
 * Initialize Foxglove client event handlers
 */
function initFoxglove() {
    foxglove.onConnect = () => {
        state.foxgloveConnected = true;
        updateStatus('foxglove', 'Connected', 'green');
        log('Foxglove connected', 'success');
    };

    foxglove.onDisconnect = () => {
        state.foxgloveConnected = false;
        updateStatus('foxglove', 'Disconnected', 'red');
        log('Foxglove disconnected', 'warn');
    };

    foxglove.onError = (error) => {
        log(`Foxglove error: ${error}`, 'error');
    };

    foxglove.onServicesAvailable = (services) => {
        state.availableServices = services;
        log(`Foxglove: ${services.length} services available`, 'info');

        // Enable service-dependent buttons if pattern service is available
        const patternStartService = getServiceName('pattern_start');
        if (patternStartService && services.some(s => s.name === patternStartService)) {
            log('Pattern services available via Foxglove', 'success');
        }
    };
}

/**
 * Connect to Foxglove Bridge
 */
function connectFoxglove() {
    if (!state.rosServerIp) {
        log('Cannot connect to Foxglove: ROS server IP not set', 'warn');
        return;
    }

    const port = state.ros2InterfaceConfig?.foxglove?.port || state.foxglovePort;
    log(`Connecting to Foxglove: ${state.rosServerIp}:${port}`, 'info');
    foxglove.connect(state.rosServerIp, port);
}

/**
 * Disconnect from Foxglove Bridge
 */
function disconnectFoxglove() {
    foxglove.disconnect();
    state.foxgloveConnected = false;
    updateStatus('foxglove', 'Disconnected', 'red');
}

/**
 * Call a ROS2 service via Foxglove (using YAML-defined service names)
 * @param {string} serviceKey - Service key from ros2_interface config
 * @param {object} request - Request object
 * @returns {Promise} Service response
 */
async function callRos2Service(serviceKey, request = {}) {
    const serviceName = getServiceName(serviceKey);
    if (!serviceName) {
        throw new Error(`Service key '${serviceKey}' not defined in ros2_interface`);
    }

    if (!state.foxgloveConnected) {
        throw new Error('Not connected to Foxglove');
    }

    return await foxglove.callService(serviceName, request);
}

/**
 * Load default robot from map_data.yaml
 */
async function loadDefaultRobot() {
    log(`Loading robots from map_data.yaml...`, 'info');
    try {
        const response = await fetch('/config/map_data.yaml');
        if (!response.ok) throw new Error(`HTTP ${response.status}`);

        const text = await response.text();
        const mapData = jsyaml.load(text);

        // robot_qr_codes에서 unique robot_type 추출
        const robotTypes = new Set();
        if (mapData.robot_qr_codes) {
            for (const qr of Object.values(mapData.robot_qr_codes)) {
                if (qr.robot_type) robotTypes.add(qr.robot_type);
            }
        }
        state.availableRobots = Array.from(robotTypes).sort();

        if (state.availableRobots.length > 0) {
            state.selectedRobot = state.availableRobots[0];
            sessionStorage.setItem('selectedRobot', state.selectedRobot);
            document.getElementById('header-robot').textContent = state.selectedRobot.toUpperCase();
            log(`Robot: ${state.selectedRobot} (default)`, 'info');
        }
    } catch (error) {
        log(`Failed to load map_data.yaml: ${error.message}`, 'warn');
    }
}

/**
 * Check if Settings page changed robot/server selection
 */
function checkSettingsChange() {
    const storedRobot = sessionStorage.getItem('selectedRobot');
    const storedServer = sessionStorage.getItem('rosServerHost');

    // Robot changed - reload ros2_interface config for new robot
    if (storedRobot && storedRobot !== state.selectedRobot) {
        state.selectedRobot = storedRobot;
        document.getElementById('header-robot').textContent = storedRobot.toUpperCase();
        log(`Robot changed to: ${storedRobot}`, 'info');

        // Reload ros2_interface config for the new robot
        loadRos2InterfaceConfig();
    }

    // Server changed - reconnect Foxglove
    if (storedServer && storedServer !== state.rosServerIp) {
        state.rosServerIp = storedServer;
        const serverName = sessionStorage.getItem('selectedServer') || storedServer;
        document.getElementById('header-server').textContent = serverName;
        log(`Server changed to: ${storedServer}`, 'info');

        // Reconnect Foxglove to new server
        if (state.foxgloveConnected) {
            disconnectFoxglove();
        }
        connectFoxglove();
    }
}

// ==================== System Control ====================

async function cleanSystem() {
    log('Cleaning system...', 'info');
    try {
        await api.cleanSystem();

        // Reset all states
        state.moveitRunning = false;
        state.controllerRunning = false;
        state.patternServiceRunning = false;
        state.tcpEndpointRunning = false;
        state.testRunning = false;
        state.worldTfRunning = false;
        state.vizLaunchRunning = false;
        state.deviceApiRunning = false;

        // Update all buttons
        updateAllButtonStates();
        updateAllStatus();

        log('System cleaned', 'success');
    } catch (error) {
        log(`Clean failed: ${error.message}`, 'error');
    }
}

async function toggleMoveIt() {
    const btn = document.getElementById('btn-moveit');
    // Get mode from sessionStorage (set by Settings page)
    const mode = sessionStorage.getItem('systemMode') || 'fake';

    try {
        if (!state.moveitRunning) {
            log(`Launching MoveIt in ${mode} mode...`, 'info');
            await api.toggleMoveIt('start', mode);
            state.moveitRunning = true;
            btn.textContent = 'Stop MoveIt';
            btn.dataset.running = 'true';
            updateStatus('moveit', 'Launching...', 'orange');
        } else {
            log('Stopping MoveIt...', 'info');
            await api.toggleMoveIt('stop');
            state.moveitRunning = false;
            btn.textContent = 'Launch MoveIt';
            btn.dataset.running = 'false';
            updateStatus('moveit', 'Stopped', 'red');
            log('MoveIt stopped', 'warn');
        }
    } catch (error) {
        log(`MoveIt error: ${error.message}`, 'error');
    }
}

async function toggleController() {
    const btn = document.getElementById('btn-controller');

    try {
        if (!state.controllerRunning) {
            log('Launching Controller...', 'info');
            await api.toggleController('start');
            state.controllerRunning = true;
            btn.textContent = 'Stop Controller';
            btn.dataset.running = 'true';
            updateStatus('controller', 'Running', 'green');
        } else {
            log('Stopping Controller...', 'info');
            await api.toggleController('stop');
            state.controllerRunning = false;
            btn.textContent = 'Launch Controller';
            btn.dataset.running = 'false';
            updateStatus('controller', 'Stopped', 'red');
            log('Controller stopped', 'warn');
        }
    } catch (error) {
        log(`Controller error: ${error.message}`, 'error');
    }
}

async function toggleWorldTf() {
    const btn = document.getElementById('btn-world-tf');

    try {
        if (!state.worldTfRunning) {
            log('Adding World TF...', 'info');
            await api.toggleWorldTf('start');
            state.worldTfRunning = true;
            btn.textContent = 'Stop World TF';
            btn.dataset.running = 'true';
            log('World TF added (critical for IK)', 'success');
        } else {
            log('Stopping World TF...', 'info');
            await api.toggleWorldTf('stop');
            state.worldTfRunning = false;
            btn.textContent = 'Add World TF';
            btn.dataset.running = 'false';
            log('World TF stopped', 'warn');
        }
    } catch (error) {
        log(`World TF error: ${error.message}`, 'error');
    }
}

async function toggleVizSystem() {
    const btn = document.getElementById('btn-viz-system');

    try {
        if (!state.vizLaunchRunning) {
            log('Launching Visualization System...', 'info');
            await api.toggleVizSystem('start');
            state.vizLaunchRunning = true;
            btn.textContent = 'Stop Viz System';
            btn.dataset.running = 'true';

            // Enable path buttons
            document.getElementById('btn-start-path').disabled = false;
            document.getElementById('btn-stop-path').disabled = false;

            log('Visualization system launched!', 'success');
        } else {
            log('Stopping Visualization System...', 'info');
            await api.toggleVizSystem('stop');
            state.vizLaunchRunning = false;
            btn.textContent = 'Visualization System';
            btn.dataset.running = 'false';

            // Disable path buttons
            document.getElementById('btn-start-path').disabled = true;
            document.getElementById('btn-stop-path').disabled = true;

            log('Visualization system stopped', 'warn');
        }
    } catch (error) {
        log(`Viz System error: ${error.message}`, 'error');
    }
}

async function startPathPlanning() {
    log('Starting path planning...', 'info');
    try {
        await api.startPathPlanning();
        log('Path planning started!', 'success');
    } catch (error) {
        log(`Path planning error: ${error.message}`, 'error');
    }
}

async function stopPathPlanning() {
    log('Stopping path planning...', 'info');
    try {
        await api.stopPathPlanning();
        log('Path planning stopped', 'success');
    } catch (error) {
        log(`Path planning error: ${error.message}`, 'error');
    }
}

// ==================== Pattern Test ====================

async function togglePatternService() {
    const btn = document.getElementById('btn-pattern-service');
    const pattern = document.getElementById('pattern-select').value;
    const size = parseFloat(document.getElementById('size-slider').value);
    const speed = parseFloat(document.getElementById('speed-slider').value);

    try {
        if (!state.patternServiceRunning) {
            log(`Launching pattern service (${pattern}, size=${size.toFixed(2)}m, speed=${speed.toFixed(1)}x)...`, 'info');
            await api.togglePatternService('start', { pattern, size, speed });
            state.patternServiceRunning = true;
            btn.textContent = 'Stop Service';
            btn.dataset.running = 'true';
            updateStatus('pattern-service', 'Running', 'green');

            // Enable test buttons
            document.getElementById('btn-preview').disabled = false;
            document.getElementById('btn-start-test').disabled = false;
        } else {
            log('Stopping pattern service...', 'info');
            await api.togglePatternService('stop');
            state.patternServiceRunning = false;
            btn.textContent = 'Launch Service';
            btn.dataset.running = 'false';
            updateStatus('pattern-service', 'Stopped', 'red');

            // Disable test buttons
            document.getElementById('btn-preview').disabled = true;
            document.getElementById('btn-start-test').disabled = true;
            document.getElementById('btn-stop-test').disabled = true;

            log('Pattern service stopped', 'warn');
        }
    } catch (error) {
        log(`Pattern service error: ${error.message}`, 'error');
    }
}

async function previewPatternTest() {
    if (!state.patternServiceRunning) {
        log('Please launch pattern service first!', 'warn');
        return;
    }

    log('Previewing pattern test (showing all target positions)...', 'info');
    try {
        await api.previewPatternTest();
        log('Pattern targets preview shown successfully', 'success');
        log('You can now see all target positions in blue (preview mode)', 'info');
    } catch (error) {
        log(`Preview failed: ${error.message}`, 'error');
    }
}

async function startPatternTest() {
    if (!state.patternServiceRunning) {
        log('Please launch pattern service first!', 'warn');
        return;
    }

    log('Starting pattern test...', 'info');
    try {
        await api.startPatternTest();
        state.testRunning = true;
        document.getElementById('btn-start-test').disabled = true;
        document.getElementById('btn-stop-test').disabled = false;
        updateStatus('test', 'Running', 'green');
        log('Pattern test started successfully', 'success');
    } catch (error) {
        log(`Start test failed: ${error.message}`, 'error');
    }
}

async function stopPatternTest() {
    log('Stopping pattern test...', 'info');
    try {
        await api.stopPatternTest();
        state.testRunning = false;
        document.getElementById('btn-start-test').disabled = false;
        document.getElementById('btn-stop-test').disabled = true;
        updateStatus('test', 'Stopped', 'red');
        log('Pattern test stopped successfully', 'success');
    } catch (error) {
        log(`Stop test failed: ${error.message}`, 'error');
    }
}

// ==================== Joint Control ====================

async function moveToHome() {
    log('Moving to home position...', 'info');
    try {
        await api.moveToHome();
        log('Moved to home position', 'success');
    } catch (error) {
        log(`Move to home failed: ${error.message}`, 'error');
    }
}

async function moveToZero() {
    log('Moving to zero position...', 'info');
    try {
        await api.moveToZero();
        log('Moved to zero position', 'success');
    } catch (error) {
        log(`Move to zero failed: ${error.message}`, 'error');
    }
}

// ==================== Status & UI ====================

function updateStatus(key, text, color) {
    const element = document.getElementById(`status-${key}`);
    if (element) {
        element.textContent = `● ${text}`;
        element.className = `status-indicator status-${color === 'green' ? 'running' : color === 'orange' ? 'warning' : 'stopped'}`;
    }
}

function updateAllStatus() {
    updateStatus('ros2', state.moveitRunning ? 'Running' : 'Stopped', state.moveitRunning ? 'green' : 'red');
    updateStatus('moveit', state.moveitRunning ? 'Running' : 'Stopped', state.moveitRunning ? 'green' : 'red');
    updateStatus('controller', state.controllerRunning ? 'Running' : 'Stopped', state.controllerRunning ? 'green' : 'red');
    updateStatus('tcp-endpoint', state.tcpEndpointRunning ? 'Running' : 'Stopped', state.tcpEndpointRunning ? 'green' : 'red');
    updateStatus('device-api', state.deviceApiRunning ? 'Running' : 'Stopped', state.deviceApiRunning ? 'green' : 'red');
    updateStatus('pattern-service', state.patternServiceRunning ? 'Running' : 'Stopped', state.patternServiceRunning ? 'green' : 'red');
    updateStatus('test', state.testRunning ? 'Running' : 'Stopped', state.testRunning ? 'green' : 'red');
    updateStatus('foxglove', state.foxgloveConnected ? 'Connected' : 'Disconnected', state.foxgloveConnected ? 'green' : 'red');
}

function updateAllButtonStates() {
    const buttons = [
        { id: 'btn-moveit', running: state.moveitRunning, textOn: 'Stop MoveIt', textOff: 'Launch MoveIt' },
        { id: 'btn-controller', running: state.controllerRunning, textOn: 'Stop Controller', textOff: 'Launch Controller' },
        { id: 'btn-world-tf', running: state.worldTfRunning, textOn: 'Stop World TF', textOff: 'Add World TF' },
        { id: 'btn-viz-system', running: state.vizLaunchRunning, textOn: 'Stop Viz System', textOff: 'Visualization System' },
        { id: 'btn-pattern-service', running: state.patternServiceRunning, textOn: 'Stop Service', textOff: 'Launch Service' }
    ];

    for (const btn of buttons) {
        const el = document.getElementById(btn.id);
        if (el) {
            el.textContent = btn.running ? btn.textOn : btn.textOff;
            el.dataset.running = btn.running.toString();
        }
    }

    // Pattern test buttons
    document.getElementById('btn-preview').disabled = !state.patternServiceRunning;
    document.getElementById('btn-start-test').disabled = !state.patternServiceRunning || state.testRunning;
    document.getElementById('btn-stop-test').disabled = !state.testRunning;

    // Path buttons
    document.getElementById('btn-start-path').disabled = !state.vizLaunchRunning;
    document.getElementById('btn-stop-path').disabled = !state.vizLaunchRunning;
}

/**
 * Start polling for system status
 */
function startStatusPolling() {
    setInterval(async () => {
        try {
            const status = await api.getStatus();
            if (status) {
                // Update states from server
                if (status.moveit !== undefined) state.moveitRunning = status.moveit;
                if (status.controller !== undefined) state.controllerRunning = status.controller;
                if (status.tcp_endpoint !== undefined) state.tcpEndpointRunning = status.tcp_endpoint;
                if (status.device_api !== undefined) state.deviceApiRunning = status.device_api;
                if (status.pattern_service !== undefined) state.patternServiceRunning = status.pattern_service;
                if (status.test_running !== undefined) state.testRunning = status.test_running;

                updateAllStatus();
                updateAllButtonStates();
            }
        } catch (error) {
            // Silently ignore polling errors
        }

        // Update last update timestamp
        document.getElementById('last-update').textContent =
            new Date().toLocaleTimeString();
    }, 5000);
}

// ==================== Output Logging (Monochrome Terminal Style) ====================

function log(message, level = 'info') {
    const outputLog = document.getElementById('output-log');
    const now = new Date();
    const timestamp = now.toTimeString().split(' ')[0]; // HH:MM:SS

    // Clean, minimal prefix - no emojis, professional style
    const prefix = {
        'info': 'INFO',
        'success': 'OK',
        'warn': 'WARN',
        'error': 'ERR',
        'debug': 'DBG'
    }[level] || 'INFO';

    // Format: [HH:MM:SS] [LEVEL] message
    const line = `[${timestamp}] [${prefix.padEnd(4)}] ${message}\n`;
    outputLog.textContent += line;

    // Auto-scroll to bottom
    outputLog.scrollTop = outputLog.scrollHeight;
}

function clearOutput() {
    document.getElementById('output-log').textContent = '';
    log('Output cleared', 'info');
}

function copyOutput() {
    const outputLog = document.getElementById('output-log');
    navigator.clipboard.writeText(outputLog.textContent)
        .then(() => log('Output copied to clipboard', 'success'))
        .catch(err => log(`Copy failed: ${err}`, 'error'));
}
