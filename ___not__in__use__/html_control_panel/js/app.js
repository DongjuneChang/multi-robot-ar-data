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
    detectedDevices: [],
    rosServerIp: null
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

    // Load configuration
    try {
        await loadConfigurations();
        await loadRobotList();
        log('Control Panel initialized successfully', 'success');
    } catch (error) {
        log(`Initialization error: ${error.message}`, 'error');
    }

    // Start status polling
    startStatusPolling();
});

/**
 * Initialize event listeners for all buttons
 */
function initEventListeners() {
    // Connection Settings
    document.getElementById('robot-type').addEventListener('change', onRobotTypeChange);
    document.getElementById('ros-server').addEventListener('change', onServerChange);

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

    // AR Devices
    document.getElementById('btn-tcp-endpoint').addEventListener('click', toggleTcpEndpoint);
    document.getElementById('btn-device-api').addEventListener('click', toggleDeviceApi);
    document.getElementById('btn-scan-devices').addEventListener('click', scanDevices);

    // Foxglove
    document.getElementById('btn-foxglove-connect').addEventListener('click', connectFoxglove);
    document.getElementById('btn-foxglove-disconnect').addEventListener('click', disconnectFoxglove);

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
 * Load device and network configurations from shared_data
 */
async function loadConfigurations() {
    try {
        // Load device config
        const deviceYaml = await api.getDeviceConfig();
        state.deviceConfig = jsyaml.load(deviceYaml);
        log('Loaded device_config.yaml', 'debug');

        // Load network config
        const networkYaml = await api.getNetworkConfig();
        state.networkConfig = jsyaml.load(networkYaml);
        log('Loaded network_config.yaml', 'debug');

        // Populate server dropdown
        populateServerList();

    } catch (error) {
        log(`Config load error: ${error.message}`, 'warn');
    }
}

/**
 * Populate ROS2 server dropdown from network_config.yaml
 */
function populateServerList() {
    const select = document.getElementById('ros-server');
    select.innerHTML = '<option value="">Select server...</option>';

    if (!state.networkConfig || !state.networkConfig.ros_config) return;

    const servers = state.networkConfig.ros_config.servers;
    for (const [name, config] of Object.entries(servers)) {
        const option = document.createElement('option');
        option.value = name;
        option.textContent = `${name} (${config.host})`;
        select.appendChild(option);
    }

    // Auto-select based on current connection
    const currentHost = state.serverHost;
    for (const [name, config] of Object.entries(servers)) {
        if (config.host === currentHost) {
            select.value = name;
            onServerChange();
            break;
        }
    }
}

/**
 * Handle server selection change
 */
function onServerChange() {
    const select = document.getElementById('ros-server');
    const serverName = select.value;

    if (!serverName || !state.networkConfig) return;

    const server = state.networkConfig.ros_config.servers[serverName];
    if (server) {
        state.rosServerIp = server.host;
        document.getElementById('connection-ip').textContent = server.host;
        document.getElementById('connection-port').textContent = server.port || 10000;
        document.getElementById('ros-server-ip').value = `${server.host}:${server.port || 10000}`;
        document.getElementById('header-server').textContent = `${serverName}:${server.host}`;

        log(`Server: ${serverName} @ ${server.host}:${server.port}`, 'info');
    }
}

/**
 * Load list of available robots
 */
async function loadRobotList() {
    try {
        const data = await api.getRobots();
        state.availableRobots = data.robots || [];

        const select = document.getElementById('robot-type');
        select.innerHTML = '';

        if (state.availableRobots.length === 0) {
            select.innerHTML = '<option value="">No robots found</option>';
            return;
        }

        for (const robot of state.availableRobots) {
            const option = document.createElement('option');
            option.value = robot;
            option.textContent = robot.toUpperCase();
            select.appendChild(option);
        }

        // Select first robot by default
        if (state.availableRobots.length > 0) {
            select.value = state.availableRobots[0];
            await onRobotTypeChange();
        }
    } catch (error) {
        log(`Failed to load robots: ${error.message}`, 'error');
        document.getElementById('robot-type').innerHTML =
            '<option value="">Error loading</option>';
    }
}

/**
 * Handle robot type selection change
 */
async function onRobotTypeChange() {
    const select = document.getElementById('robot-type');
    state.selectedRobot = select.value;

    if (!state.selectedRobot) return;

    log(`Loading ${state.selectedRobot} config...`, 'info');

    try {
        // Load robot.yaml
        const robotYaml = await api.getRobotConfig(state.selectedRobot);
        state.robotConfig = jsyaml.load(robotYaml);

        // Load frames.yaml
        const framesYaml = await api.getRobotFrames(state.selectedRobot);
        state.robotFrames = jsyaml.load(framesYaml);

        // Update header
        document.getElementById('header-robot').textContent = state.selectedRobot.toUpperCase();

        log(`Robot: ${state.selectedRobot}`, 'success');
    } catch (error) {
        log(`Failed to load robot config: ${error.message}`, 'error');
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
    const mode = document.querySelector('input[name="mode"]:checked').value;

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

// ==================== AR Devices ====================

async function toggleTcpEndpoint() {
    const btn = document.getElementById('btn-tcp-endpoint');

    try {
        if (!state.tcpEndpointRunning) {
            log('Launching TCP Endpoint...', 'info');
            await api.toggleTcpEndpoint('start');
            state.tcpEndpointRunning = true;
            btn.textContent = 'Stop TCP Endpoint';
            btn.dataset.running = 'true';
            updateStatus('tcp-endpoint', 'Running', 'green');
            log(`TCP Endpoint running on port 10000`, 'success');
        } else {
            log('Stopping TCP Endpoint...', 'info');
            await api.toggleTcpEndpoint('stop');
            state.tcpEndpointRunning = false;
            btn.textContent = 'Launch TCP Endpoint';
            btn.dataset.running = 'false';
            updateStatus('tcp-endpoint', 'Stopped', 'red');
            log('TCP Endpoint stopped', 'warn');
        }
    } catch (error) {
        log(`TCP Endpoint error: ${error.message}`, 'error');
    }
}

async function toggleDeviceApi() {
    const btn = document.getElementById('btn-device-api');

    try {
        if (!state.deviceApiRunning) {
            log('Starting Device API server...', 'info');
            await api.toggleDeviceApi('start');
            state.deviceApiRunning = true;
            btn.textContent = 'Stop Device API';
            btn.dataset.running = 'true';
            updateStatus('device-api', 'Running', 'green');
            log('Device API server started', 'success');
        } else {
            log('Stopping Device API server...', 'info');
            await api.toggleDeviceApi('stop');
            state.deviceApiRunning = false;
            btn.textContent = 'Start Device API';
            btn.dataset.running = 'false';
            updateStatus('device-api', 'Stopped', 'red');
            log('Device API server stopped', 'warn');
        }
    } catch (error) {
        log(`Device API error: ${error.message}`, 'error');
    }
}

async function scanDevices() {
    log('Scanning for AR devices (HoloLens/Quest)...', 'info');
    const btn = document.getElementById('btn-scan-devices');
    btn.disabled = true;
    btn.textContent = 'Scanning...';

    try {
        // First trigger scan
        await api.scanDevices();

        // Wait a moment then get results
        await new Promise(resolve => setTimeout(resolve, 1000));
        const data = await api.getDevices();

        state.detectedDevices = data.devices || {};
        const count = Object.keys(state.detectedDevices).length;

        // Update device list
        updateDeviceList(state.detectedDevices);

        // Update status
        if (count > 0) {
            updateStatus('ar-devices', `${count} found`, 'green');
            log(`Found ${count} AR device(s)`, 'success');

            // Update ROS Server IP
            if (data.server_ip) {
                state.rosServerIp = data.server_ip;
                document.getElementById('ros-server-ip').value =
                    `${state.rosServerIp}:10000`;
                updateStatus('ros-ip', `${state.rosServerIp}:10000`, 'green');
            }
        } else {
            updateStatus('ar-devices', 'None found', 'orange');
            log('No AR devices found on network', 'warn');
        }
    } catch (error) {
        updateStatus('ar-devices', 'Scan failed', 'red');
        log(`Device scan failed: ${error.message}`, 'error');
    } finally {
        btn.disabled = false;
        btn.textContent = 'Scan Devices';
    }
}

function updateDeviceList(devices) {
    const container = document.getElementById('device-list');

    if (Object.keys(devices).length === 0) {
        container.innerHTML = '<p class="no-devices">No devices found</p>';
        return;
    }

    let html = '<table class="device-table"><thead><tr>' +
        '<th>Device</th><th>IP</th><th>Type</th><th>Status</th>' +
        '</tr></thead><tbody>';

    for (const [ip, device] of Object.entries(devices)) {
        const hostname = device.hostname || 'Unknown';
        const tags = device.tags || [];
        let deviceType = 'Unknown';

        if (tags.includes('HoloLens2')) deviceType = 'HoloLens';
        else if (tags.includes('MetaQuest')) deviceType = 'Meta Quest';

        html += `<tr>
            <td>${hostname}</td>
            <td>${ip}</td>
            <td>${deviceType}</td>
            <td><span class="status-ready">Ready</span></td>
        </tr>`;
    }

    html += '</tbody></table>';
    container.innerHTML = html;
}

// ==================== Foxglove ====================

function connectFoxglove() {
    const host = state.rosServerIp || state.serverHost;
    log(`Connecting to Foxglove Bridge at ${host}:${state.foxglovePort}...`, 'info');

    foxglove.onConnect = () => {
        state.foxgloveConnected = true;
        updateStatus('foxglove', 'Connected', 'green');
        document.getElementById('foxglove-status').textContent = 'Connected';
        document.getElementById('foxglove-status').className = 'status-indicator status-running';
        document.getElementById('btn-foxglove-connect').disabled = true;
        document.getElementById('btn-foxglove-disconnect').disabled = false;
        log('Foxglove Bridge connected', 'success');

        // Subscribe to joint states
        setTimeout(() => {
            const topics = foxglove.getAvailableTopics();
            log(`Available topics: ${topics.map(t => t.topic).join(', ')}`, 'debug');

            // Subscribe to /joint_states if available
            foxglove.subscribe('/joint_states', (msg) => {
                const formatted = ROS2MessageParser.formatJointAngles(msg.data, true);
                document.getElementById('joint-states-data').textContent = formatted;
            });
        }, 1000);
    };

    foxglove.onDisconnect = () => {
        state.foxgloveConnected = false;
        updateStatus('foxglove', 'Disconnected', 'red');
        document.getElementById('foxglove-status').textContent = 'Disconnected';
        document.getElementById('foxglove-status').className = 'status-indicator status-stopped';
        document.getElementById('btn-foxglove-connect').disabled = false;
        document.getElementById('btn-foxglove-disconnect').disabled = true;
        document.getElementById('joint-states-data').textContent = 'No data';
        log('Foxglove Bridge disconnected', 'warn');
    };

    foxglove.onError = (error) => {
        log(`Foxglove error: ${error}`, 'error');
    };

    foxglove.connect(host, state.foxglovePort);
}

function disconnectFoxglove() {
    log('Disconnecting from Foxglove Bridge...', 'info');
    foxglove.disconnect();
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
        { id: 'btn-pattern-service', running: state.patternServiceRunning, textOn: 'Stop Service', textOff: 'Launch Service' },
        { id: 'btn-tcp-endpoint', running: state.tcpEndpointRunning, textOn: 'Stop TCP Endpoint', textOff: 'Launch TCP Endpoint' },
        { id: 'btn-device-api', running: state.deviceApiRunning, textOn: 'Stop Device API', textOff: 'Start Device API' }
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
