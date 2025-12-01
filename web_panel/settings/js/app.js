/**
 * Settings Panel - Tab-based Robot Configuration
 * Each robot has its own tab with: QR Info, Config, Foxglove, AR Device, Admittance
 */

// State
let robots = [];
let robotConfigs = {};
let robotQRs = {};
let selectedRobot = null;
let foxgloveConnected = false;

// Project State
let currentProject = null;
let showAllRobots = false;

// ==================== Project Loading ====================

function loadCurrentProject() {
    const projectJson = sessionStorage.getItem('currentProject');
    if (projectJson) {
        try {
            currentProject = JSON.parse(projectJson);
            console.log('[Settings] Loaded project:', currentProject.name || currentProject.id);
            return currentProject;
        } catch (e) {
            console.warn('[Settings] Failed to parse project:', e);
        }
    }
    return null;
}

function getProjectRobots() {
    if (!currentProject || !currentProject.robots) return [];
    return currentProject.robots.map(r => ({
        type: r.type,
        namespace: r.namespace,
        mode: r.mode
    }));
}

function applyProjectSettings() {
    if (!currentProject) return;

    // Apply server from project
    if (currentProject.server?.ros2_server) {
        const serverSelect = document.getElementById('ros-server');
        if (serverSelect) {
            serverSelect.value = currentProject.server.ros2_server;
            onServerChange();
            console.log('[Settings] Applied project server:', currentProject.server.ros2_server);
        }
    }

    // Apply mode from first robot (if available)
    const projectRobots = getProjectRobots();
    if (projectRobots.length > 0) {
        const mode = projectRobots[0].mode?.toLowerCase() || 'fake';
        const modeRadio = document.querySelector(`input[name="mode"][value="${mode}"]`);
        if (modeRadio) {
            modeRadio.checked = true;
            sessionStorage.setItem('systemMode', mode);
            console.log('[Settings] Applied project mode:', mode);
        }
    }
}

// ==================== Initialization ====================

document.addEventListener('DOMContentLoaded', async () => {
    console.log('[Settings] Initializing tab-based Robot Settings...');

    // Load current project from session
    loadCurrentProject();

    // Cache server list first (for per-robot connection settings)
    await cacheServerList();

    // Load server options (for global/default display)
    await loadServerOptions();

    // Apply project settings if available
    applyProjectSettings();

    // Load robots and create tabs
    await loadRobotsAndCreateTabs();

    // Init mode selection (global default)
    initModeSelection();

    // Periodic updates
    setInterval(updateTimestamp, 1000);

    // Add project indicator to header if project loaded
    if (currentProject) {
        addProjectIndicator();
    }

    console.log('[Settings] Initialized');
});

function addProjectIndicator() {
    const header = document.querySelector('.global-settings');
    if (!header || document.getElementById('project-indicator')) return;

    const indicator = document.createElement('div');
    indicator.id = 'project-indicator';
    indicator.className = 'global-item';
    indicator.innerHTML = `
        <span class="label">Project:</span>
        <span class="value" style="color: #4CAF50; font-weight: bold;">${currentProject.name || currentProject.id}</span>
        <label style="margin-left: 10px;">
            <input type="checkbox" id="show-all-robots" ${showAllRobots ? 'checked' : ''}>
            Show All
        </label>
    `;
    header.appendChild(indicator);

    // Toggle handler
    document.getElementById('show-all-robots').addEventListener('change', async (e) => {
        showAllRobots = e.target.checked;
        await loadRobotsAndCreateTabs();
    });
}

// ==================== Server Selection ====================

async function loadServerOptions() {
    try {
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

        serverSelect.addEventListener('change', onServerChange);

        // Restore saved value
        const storedServer = sessionStorage.getItem('selectedServer');
        if (storedServer) serverSelect.value = storedServer;

        onServerChange();

    } catch (e) {
        console.error('[Settings] Failed to load server options:', e);
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

        sessionStorage.setItem('selectedServer', serverSelect.value);
        sessionStorage.setItem('rosServerHost', host);
        sessionStorage.setItem('rosServerPort', port);

        log(`[Settings] Server: ${serverSelect.value} (${host}:${port})`);
    }
}

function getSelectedServer() {
    return document.getElementById('ros-server')?.value || null;
}

function getServerHost() {
    const serverSelect = document.getElementById('ros-server');
    return serverSelect?.selectedOptions[0]?.dataset.host || 'localhost';
}

// ==================== Mode Selection ====================

function initModeSelection() {
    const modeRadios = document.querySelectorAll('input[name="mode"]');
    const savedMode = sessionStorage.getItem('systemMode') || 'fake';

    modeRadios.forEach(radio => {
        if (radio.value === savedMode) radio.checked = true;
        radio.addEventListener('change', () => {
            sessionStorage.setItem('systemMode', radio.value);
            log(`[Settings] Mode: ${radio.value}`);
        });
    });
}

// ==================== Robot Tabs ====================

// Manufacturer display names and order
const MANUFACTURER_INFO = {
    'UFACTORY': { displayName: 'UFACTORY', order: 1 },
    'Universal Robots': { displayName: 'Universal Robots', order: 2 },
    'KUKA': { displayName: 'KUKA', order: 3 },
    'Franka Emika': { displayName: 'Franka', order: 4 },
    'Kinova': { displayName: 'Kinova', order: 5 }
};

// Store robot-manufacturer mapping and UI state
let robotManufacturers = {};
let expandedGroups = {};

async function loadRobotsAndCreateTabs() {
    try {
        const resp = await api.getRobots();
        let allRobots = resp.robots || [];

        if (allRobots.length === 0) {
            document.getElementById('robot-tabs').innerHTML = '<div class="tab-error">No robots found</div>';
            return;
        }

        // Filter by project if project loaded and not showing all
        const projectRobots = getProjectRobots();
        if (currentProject && !showAllRobots && projectRobots.length > 0) {
            const projectRobotTypes = projectRobots.map(r => r.type);
            robots = allRobots.filter(r => projectRobotTypes.includes(r));
            console.log('[Settings] Filtered to project robots:', robots);
        } else {
            robots = allRobots;
        }

        if (robots.length === 0) {
            document.getElementById('robot-tabs').innerHTML = '<div class="tab-error">No robots in project</div>';
            return;
        }

        // Show loading state
        document.getElementById('robot-tabs').innerHTML = '<div class="tab-loading">Loading robots...</div>';

        // Load manufacturer info for each robot (in parallel)
        const configPromises = robots.map(async (robot) => {
            try {
                const config = await api.getRobotConfig(robot);
                robotConfigs[robot] = config;
                robotManufacturers[robot] = config.robot?.manufacturer || 'Unknown';
            } catch (e) {
                console.warn(`[Settings] Could not load config for ${robot}:`, e);
                robotManufacturers[robot] = 'Unknown';
            }
        });
        await Promise.all(configPromises);

        // Group robots by manufacturer
        const groups = groupRobotsByManufacturer(robots);

        // Load first robot or stored selection
        const storedRobot = sessionStorage.getItem('selectedRobot');
        const initialRobot = (storedRobot && robots.includes(storedRobot)) ? storedRobot : robots[0];

        // Expand the group containing the initial robot
        const initialManufacturer = robotManufacturers[initialRobot];
        if (initialManufacturer) {
            expandedGroups[initialManufacturer] = true;
        }

        // Render grouped tabs
        renderGroupedTabs(groups);

        await selectRobotTab(initialRobot);

    } catch (e) {
        console.error('[Settings] Failed to load robots:', e);
        document.getElementById('robot-tabs').innerHTML = '<div class="tab-error">Failed to load robots</div>';
    }
}

function groupRobotsByManufacturer(robotList) {
    const groups = {};

    for (const robot of robotList) {
        const manufacturer = robotManufacturers[robot] || 'Unknown';
        if (!groups[manufacturer]) {
            groups[manufacturer] = [];
        }
        groups[manufacturer].push(robot);
    }

    // Sort groups by defined order
    const sortedGroups = {};
    const sortedKeys = Object.keys(groups).sort((a, b) => {
        const orderA = MANUFACTURER_INFO[a]?.order || 99;
        const orderB = MANUFACTURER_INFO[b]?.order || 99;
        return orderA - orderB;
    });

    for (const key of sortedKeys) {
        sortedGroups[key] = groups[key];
    }

    return sortedGroups;
}

function renderGroupedTabs(groups) {
    const tabsContainer = document.getElementById('robot-tabs');
    tabsContainer.innerHTML = '';

    for (const [manufacturer, robotList] of Object.entries(groups)) {
        const isExpanded = expandedGroups[manufacturer] === true;
        const displayName = MANUFACTURER_INFO[manufacturer]?.displayName || manufacturer;

        // Create group container
        const groupDiv = document.createElement('div');
        groupDiv.className = 'tab-group';
        groupDiv.dataset.manufacturer = manufacturer;

        // Create group header (clickable)
        const groupHeader = document.createElement('button');
        groupHeader.className = `tab-group-header ${isExpanded ? 'expanded' : ''}`;
        groupHeader.innerHTML = `
            <span class="group-icon">${isExpanded ? '▼' : '▶'}</span>
            <span class="group-name">${displayName}</span>
            <span class="group-count">(${robotList.length})</span>
        `;
        groupHeader.addEventListener('click', () => toggleGroup(manufacturer));
        groupDiv.appendChild(groupHeader);

        // Create robot tabs container (collapsible)
        const robotsDiv = document.createElement('div');
        robotsDiv.className = `tab-group-robots ${isExpanded ? 'expanded' : ''}`;

        for (const robot of robotList) {
            const tab = document.createElement('button');
            tab.className = `robot-tab ${selectedRobot === robot ? 'active' : ''}`;
            tab.textContent = robot;
            tab.dataset.robot = robot;
            tab.addEventListener('click', () => selectRobotTab(robot));
            robotsDiv.appendChild(tab);
        }

        groupDiv.appendChild(robotsDiv);
        tabsContainer.appendChild(groupDiv);
    }
}

function toggleGroup(manufacturer) {
    expandedGroups[manufacturer] = !expandedGroups[manufacturer];
    const groups = groupRobotsByManufacturer(robots);
    renderGroupedTabs(groups);
}

async function selectRobotTab(robotType) {
    // Update tab active state
    document.querySelectorAll('.robot-tab').forEach(tab => {
        tab.classList.toggle('active', tab.dataset.robot === robotType);
    });

    selectedRobot = robotType;
    sessionStorage.setItem('selectedRobot', robotType);
    document.getElementById('config-status').textContent = `Loading ${robotType}...`;

    // Load robot data if not cached
    if (!robotConfigs[robotType]) {
        try {
            robotConfigs[robotType] = await api.getRobotConfig(robotType);
        } catch (e) {
            console.error(`[Settings] Failed to load config for ${robotType}:`, e);
        }
    }

    if (!robotQRs[robotType]) {
        try {
            const qrResp = await api.request(`/api/robots/${robotType}/qr`);
            robotQRs[robotType] = qrResp.qr_codes || {};
        } catch (e) {
            console.error(`[Settings] Failed to load QR for ${robotType}:`, e);
            robotQRs[robotType] = {};
        }
    }

    // Render robot panel
    renderRobotPanel(robotType);
    document.getElementById('config-status').textContent = `Loaded: ${robotType}`;
}

// ==================== Robot Panel Rendering ====================

function renderRobotPanel(robotType) {
    const config = robotConfigs[robotType] || {};
    const qrCodes = robotQRs[robotType] || {};

    const container = document.getElementById('tab-content');
    container.innerHTML = `
        <div class="robot-panel" data-robot="${robotType}">
            <!-- Robot Header -->
            <div class="robot-header">
                <h2>${robotType.toUpperCase()}</h2>
                <span class="manufacturer">${config.robot?.manufacturer || '-'}</span>
            </div>

            <div class="panel-grid">
                <!-- QR Code Info -->
                <section class="panel qr-panel">
                    <h3>QR Mapping</h3>
                    ${renderQRInfo(qrCodes)}
                </section>

                <!-- Connection Settings (Per-Robot) -->
                <section class="panel connection-panel">
                    <h3>Connection Settings</h3>
                    ${renderConnectionSettings(robotType)}
                </section>

                <!-- Robot Configuration -->
                <section class="panel config-panel">
                    <h3>Robot Configuration</h3>
                    <div class="info-row">
                        <span class="info-label">DOF:</span>
                        <span>${config.robot?.dof || '-'}</span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">Base Frame:</span>
                        <span class="mono">${getBaseFrame(config)}</span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">EEF Frame:</span>
                        <span class="mono">${getEEFFrame(config)}</span>
                    </div>
                    ${renderJointTable(config)}
                </section>

                <!-- Foxglove Bridge -->
                <section class="panel foxglove-panel">
                    <h3>Foxglove Bridge</h3>
                    <div class="connection-info">
                        <span class="label">WebSocket:</span>
                        <span class="value" id="foxglove-url">ws://${getServerHost()}:8765</span>
                    </div>
                    <div class="connection-info">
                        <span class="label">Status:</span>
                        <span class="status-dot disconnected" id="foxglove-status-dot"></span>
                        <span id="foxglove-status">Disconnected</span>
                    </div>
                    <div class="button-row">
                        <button id="btn-foxglove-connect" class="btn btn-primary" onclick="connectFoxglove()">Connect</button>
                        <button id="btn-foxglove-disconnect" class="btn btn-secondary" onclick="disconnectFoxglove()" disabled>Disconnect</button>
                    </div>
                    <div class="foxglove-topics" id="foxglove-topics">
                        <div class="no-data">Not connected</div>
                    </div>
                </section>

                <!-- AR Device -->
                <section class="panel device-panel">
                    <h3>AR Device</h3>
                    <div class="button-row">
                        <button id="btn-tcp-endpoint" class="btn btn-primary" data-running="false" onclick="toggleTcpEndpoint()">TCP Endpoint</button>
                        <button id="btn-device-api" class="btn btn-primary" data-running="false" onclick="toggleDeviceApi()">Device API</button>
                    </div>
                    <div class="connection-info">
                        <span class="label">TCP Status:</span>
                        <span class="status-dot disconnected" id="tcp-status-dot"></span>
                        <span id="tcp-status">Stopped</span>
                    </div>
                    <div class="button-row">
                        <button id="btn-scan-devices" class="btn btn-secondary" onclick="scanDevices()">Scan Devices</button>
                    </div>
                    <div class="device-list" id="device-list">
                        <p class="no-devices">No devices scanned</p>
                    </div>
                </section>

                <!-- Admittance Control -->
                <section class="panel admittance-panel">
                    <h3>Admittance Control</h3>
                    ${renderAdmittanceControl(config)}
                </section>

                <!-- F/T Sensor -->
                <section class="panel ft-sensor-panel">
                    <h3>F/T Sensor</h3>
                    ${renderFTSensor(config)}
                </section>
            </div>

            <!-- Actions -->
            <div class="panel-actions">
                <button class="btn btn-primary" onclick="applySettings()">Apply Settings</button>
                <button class="btn btn-secondary" onclick="resetSettings()">Reset to Defaults</button>
            </div>
        </div>
    `;
}

function renderQRInfo(qrCodes) {
    const entries = Object.entries(qrCodes);
    if (entries.length === 0) {
        return '<div class="no-data">No QR codes assigned</div>';
    }

    let html = '';
    for (const [qrId, info] of entries) {
        html += `
            <div class="qr-item">
                <div class="qr-id">${qrId}</div>
                <div class="qr-details">
                    <div class="info-row">
                        <span class="info-label">Location:</span>
                        <input type="text" class="readonly-input" value="${info.location || '-'}" readonly>
                    </div>
                    <div class="info-row">
                        <span class="info-label">Mode:</span>
                        <input type="text" class="readonly-input" value="${info.mode || 'standalone'}" readonly>
                    </div>
                    ${info.teleoperation_pair ? `
                    <div class="info-row">
                        <span class="info-label">Pair:</span>
                        <input type="text" class="readonly-input" value="${info.teleoperation_pair}" readonly>
                    </div>` : ''}
                </div>
            </div>
        `;
    }
    return html;
}

function renderJointTable(config) {
    if (!config.joints?.names || config.joints.names.length === 0) {
        return '<div class="no-data">No joint data</div>';
    }

    const names = config.joints.names;
    const positions = config.joints.positions || {};
    const zero = positions.zero || [];
    const home = positions.home || [];
    const limits = config.joints.limits || {};

    let html = `
        <table class="joint-table">
            <thead>
                <tr>
                    <th>Joint</th>
                    <th>Zero</th>
                    <th>Home</th>
                    <th>Min</th>
                    <th>Max</th>
                </tr>
            </thead>
            <tbody>
    `;

    for (let i = 0; i < names.length; i++) {
        const name = names[i];
        const limit = limits[name] || [-360, 360];
        html += `
            <tr>
                <td>${name}</td>
                <td>${zero[i] !== undefined ? zero[i] : '-'}</td>
                <td>${home[i] !== undefined ? home[i] : '-'}</td>
                <td>${limit[0]}</td>
                <td>${limit[1]}</td>
            </tr>
        `;
    }

    html += '</tbody></table>';
    return html;
}

function renderAdmittanceControl(config) {
    const admittance = config.admittance || {};
    const dynamics = admittance.dynamics || {};
    const inertia = dynamics.inertia || [10.0, 10.0, 10.0];
    const damping = dynamics.damping || [3.0, 3.0, 3.0];
    const stiffness = dynamics.stiffness_p || [0.0, 0.0, 0.0];

    return `
        <div class="control-group">
            <label><input type="checkbox" id="admittance-enabled" checked> Enable Admittance Control</label>
        </div>
        <div class="admittance-params">
            <div class="param-group">
                <label>Inertia [x, y, z] (kg):</label>
                <div class="xyz-inputs">
                    <input type="number" id="mass-x" step="0.1" value="${inertia[0]}">
                    <input type="number" id="mass-y" step="0.1" value="${inertia[1]}">
                    <input type="number" id="mass-z" step="0.1" value="${inertia[2]}">
                </div>
            </div>
            <div class="param-group">
                <label>Damping [x, y, z]:</label>
                <div class="xyz-inputs">
                    <input type="number" id="damping-x" step="0.1" value="${damping[0]}">
                    <input type="number" id="damping-y" step="0.1" value="${damping[1]}">
                    <input type="number" id="damping-z" step="0.1" value="${damping[2]}">
                </div>
            </div>
            <div class="param-group">
                <label>Stiffness [x, y, z]:</label>
                <div class="xyz-inputs">
                    <input type="number" id="stiffness-x" step="0.1" value="${stiffness[0]}">
                    <input type="number" id="stiffness-y" step="0.1" value="${stiffness[1]}">
                    <input type="number" id="stiffness-z" step="0.1" value="${stiffness[2]}">
                </div>
            </div>
        </div>
    `;
}

function renderFTSensor(config) {
    const ftSensor = config.ft_sensor || {};
    const forceThreshold = ftSensor.force_threshold || [50.0, 50.0, 50.0];
    const torqueThreshold = ftSensor.torque_threshold || [10.0, 10.0, 10.0];
    const enabled = ftSensor.enabled !== false;

    return `
        <div class="control-group">
            <label><input type="checkbox" id="ft-sensor-enabled" ${enabled ? 'checked' : ''}> Enable F/T Sensor</label>
        </div>
        <div class="ft-sensor-params">
            <div class="param-group">
                <label>Force Threshold [x, y, z] (N):</label>
                <div class="xyz-inputs">
                    <input type="number" id="force-x" step="1" value="${forceThreshold[0]}">
                    <input type="number" id="force-y" step="1" value="${forceThreshold[1]}">
                    <input type="number" id="force-z" step="1" value="${forceThreshold[2]}">
                </div>
            </div>
            <div class="param-group">
                <label>Torque Threshold [x, y, z] (Nm):</label>
                <div class="xyz-inputs">
                    <input type="number" id="torque-x" step="0.5" value="${torqueThreshold[0]}">
                    <input type="number" id="torque-y" step="0.5" value="${torqueThreshold[1]}">
                    <input type="number" id="torque-z" step="0.5" value="${torqueThreshold[2]}">
                </div>
            </div>
        </div>
    `;
}

function getBaseFrame(config) {
    const networkFrames = config?.frame_hierarchy?.network_frames || {};
    for (const [key, value] of Object.entries(networkFrames)) {
        if (key.includes('base')) return value.ros_frame || '-';
    }
    return '-';
}

function getEEFFrame(config) {
    const networkFrames = config?.frame_hierarchy?.network_frames || {};
    for (const [key, value] of Object.entries(networkFrames)) {
        if (key.includes('eef')) return value.ros_frame || '-';
    }
    return '-';
}

// ==================== Connection Settings (Per-Robot) ====================

function renderConnectionSettings(robotType) {
    // Get stored values for this robot, or use defaults
    const storedSettings = getRobotConnectionSettings(robotType);
    const servers = getServerList();

    let serverOptions = '';
    for (const [name, info] of Object.entries(servers)) {
        const selected = storedSettings.server === name ? 'selected' : '';
        serverOptions += `<option value="${name}" ${selected}>${name} (${info.host}:${info.port})</option>`;
    }

    return `
        <div class="control-group">
            <label for="robot-server-${robotType}">ROS2 Server:</label>
            <select id="robot-server-${robotType}" class="robot-setting" data-robot="${robotType}" data-field="server" onchange="saveRobotConnectionSetting('${robotType}', 'server', this.value)">
                ${serverOptions}
            </select>
        </div>
        <div class="control-group">
            <label>Mode:</label>
            <div class="mode-group-inline">
                <label><input type="radio" name="robot-mode-${robotType}" value="fake" ${storedSettings.mode === 'fake' ? 'checked' : ''} onchange="saveRobotConnectionSetting('${robotType}', 'mode', 'fake')"> Fake</label>
                <label><input type="radio" name="robot-mode-${robotType}" value="real" ${storedSettings.mode === 'real' ? 'checked' : ''} onchange="saveRobotConnectionSetting('${robotType}', 'mode', 'real')"> Real</label>
                <label><input type="radio" name="robot-mode-${robotType}" value="gazebo" ${storedSettings.mode === 'gazebo' ? 'checked' : ''} onchange="saveRobotConnectionSetting('${robotType}', 'mode', 'gazebo')"> Gazebo</label>
            </div>
        </div>
        <div class="control-group">
            <label for="robot-domain-${robotType}">ROS_DOMAIN_ID:</label>
            <input type="number" id="robot-domain-${robotType}" min="0" max="232" value="${storedSettings.ros_domain_id}"
                   onchange="saveRobotConnectionSetting('${robotType}', 'ros_domain_id', this.value)">
        </div>
        <div class="control-group">
            <label for="robot-namespace-${robotType}">Namespace:</label>
            <input type="text" id="robot-namespace-${robotType}" value="${storedSettings.namespace}" placeholder="e.g., robot1"
                   onchange="saveRobotConnectionSetting('${robotType}', 'namespace', this.value)">
        </div>
    `;
}

// Storage key for robot connection settings
function getRobotConnectionKey(robotType) {
    return `robotConnection_${robotType}`;
}

// Get stored connection settings for a robot
function getRobotConnectionSettings(robotType) {
    const stored = sessionStorage.getItem(getRobotConnectionKey(robotType));
    if (stored) {
        return JSON.parse(stored);
    }
    // Default values
    return {
        server: document.getElementById('ros-server')?.value || '',
        mode: 'fake',
        ros_domain_id: 0,
        namespace: ''
    };
}

// Save a single connection setting for a robot
function saveRobotConnectionSetting(robotType, field, value) {
    const settings = getRobotConnectionSettings(robotType);
    settings[field] = field === 'ros_domain_id' ? parseInt(value) : value;
    sessionStorage.setItem(getRobotConnectionKey(robotType), JSON.stringify(settings));
    console.log(`[Settings] ${robotType}.${field} = ${value}`);
}

// Get server list from cached network config
let cachedServers = {};
function getServerList() {
    return cachedServers;
}

// Cache servers after loading
async function cacheServerList() {
    try {
        const networkConfig = await api.getNetworkConfig();
        if (networkConfig) {
            const parsed = typeof networkConfig === 'string' ? jsyaml.load(networkConfig) : networkConfig;
            cachedServers = parsed.servers?.ros2 || {};
        }
    } catch (e) {
        console.error('[Settings] Failed to cache server list:', e);
    }
}

// ==================== Foxglove ====================

function connectFoxglove() {
    const host = getServerHost();
    const port = 8765;

    console.log(`[Settings] Connecting to Foxglove at ${host}:${port}`);

    if (typeof foxglove === 'undefined') {
        log('[Settings] Foxglove module not loaded', 'error');
        return;
    }

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
        log(`[Settings] Foxglove failed: ${host}:${port}`, 'error');
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
    for (const topic of topics.slice(0, 15)) {
        html += `<div class="topic-item">
            <span class="topic-name">${topic.topic}</span>
            <span class="topic-type">${topic.schemaName || '-'}</span>
        </div>`;
    }

    if (topics.length > 15) {
        html += `<div class="topic-item">... and ${topics.length - 15} more</div>`;
    }

    container.innerHTML = html;
}

// ==================== ROS2 Services ====================

async function toggleTcpEndpoint() {
    const btn = document.getElementById('btn-tcp-endpoint');
    const isRunning = btn.dataset.running === 'true';
    const action = isRunning ? 'stop' : 'start';
    const server = getSelectedServer();
    const host = getServerHost();

    try {
        log(`[Settings] ${action === 'start' ? 'Starting' : 'Stopping'} TCP Endpoint on ${host}...`);
        const result = await api.toggleTcpEndpoint(action, server);

        if (result.status === 'started' || result.status === 'success') {
            btn.dataset.running = 'true';
            btn.textContent = 'Stop TCP';
            btn.classList.replace('btn-primary', 'btn-danger');
            updateTcpStatus('Running', true);
            log(`[Settings] TCP Endpoint started on ${host}`);
        } else if (result.status === 'stopped') {
            btn.dataset.running = 'false';
            btn.textContent = 'TCP Endpoint';
            btn.classList.replace('btn-danger', 'btn-primary');
            updateTcpStatus('Stopped', false);
            log('[Settings] TCP Endpoint stopped');
        } else {
            log(`[Settings] TCP Endpoint: ${result.error || 'unknown'}`, 'error');
        }
    } catch (e) {
        log(`[Settings] TCP Endpoint error: ${e.message}`, 'error');
    }
}

async function toggleDeviceApi() {
    const btn = document.getElementById('btn-device-api');
    const isRunning = btn.dataset.running === 'true';
    const action = isRunning ? 'stop' : 'start';
    const server = getSelectedServer();
    const host = getServerHost();

    try {
        log(`[Settings] ${action === 'start' ? 'Starting' : 'Stopping'} Device API on ${host}...`);
        const result = await api.toggleDeviceApi(action, server);

        if (result.status === 'started' || result.status === 'success') {
            btn.dataset.running = 'true';
            btn.textContent = 'Stop API';
            btn.classList.replace('btn-primary', 'btn-danger');
            log(`[Settings] Device API started on ${host}`);
        } else if (result.status === 'stopped') {
            btn.dataset.running = 'false';
            btn.textContent = 'Device API';
            btn.classList.replace('btn-danger', 'btn-primary');
            log('[Settings] Device API stopped');
        } else {
            log(`[Settings] Device API: ${result.error || 'unknown'}`, 'error');
        }
    } catch (e) {
        log(`[Settings] Device API error: ${e.message}`, 'error');
    }
}

async function scanDevices() {
    const btn = document.getElementById('btn-scan-devices');
    const deviceList = document.getElementById('device-list');
    const server = getSelectedServer();
    const host = getServerHost();

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
        deviceList.innerHTML = '<p class="no-devices">Scan failed</p>';
        log(`[Settings] Device scan error: ${e.message}`, 'error');
    } finally {
        btn.disabled = false;
        btn.textContent = 'Scan Devices';
    }
}

function updateTcpStatus(text, connected) {
    const statusEl = document.getElementById('tcp-status');
    const dotEl = document.getElementById('tcp-status-dot');
    if (statusEl) statusEl.textContent = text;
    if (dotEl) dotEl.className = connected ? 'status-dot connected' : 'status-dot disconnected';
}

// ==================== Settings Actions ====================

async function applySettings() {
    const settings = {
        robot: selectedRobot,
        admittance: {
            enabled: document.getElementById('admittance-enabled')?.checked ?? true,
            dynamics: {
                inertia: [
                    parseFloat(document.getElementById('mass-x')?.value || 10.0),
                    parseFloat(document.getElementById('mass-y')?.value || 10.0),
                    parseFloat(document.getElementById('mass-z')?.value || 10.0)
                ],
                damping: [
                    parseFloat(document.getElementById('damping-x')?.value || 3.0),
                    parseFloat(document.getElementById('damping-y')?.value || 3.0),
                    parseFloat(document.getElementById('damping-z')?.value || 3.0)
                ],
                stiffness_p: [
                    parseFloat(document.getElementById('stiffness-x')?.value || 0.0),
                    parseFloat(document.getElementById('stiffness-y')?.value || 0.0),
                    parseFloat(document.getElementById('stiffness-z')?.value || 0.0)
                ]
            }
        },
        ft_sensor: {
            enabled: document.getElementById('ft-sensor-enabled')?.checked ?? true,
            force_threshold: [
                parseFloat(document.getElementById('force-x')?.value || 50.0),
                parseFloat(document.getElementById('force-y')?.value || 50.0),
                parseFloat(document.getElementById('force-z')?.value || 50.0)
            ],
            torque_threshold: [
                parseFloat(document.getElementById('torque-x')?.value || 10.0),
                parseFloat(document.getElementById('torque-y')?.value || 10.0),
                parseFloat(document.getElementById('torque-z')?.value || 10.0)
            ]
        }
    };

    console.log('[Settings] Applying:', settings);
    sessionStorage.setItem('admittanceSettings', JSON.stringify(settings.admittance));
    sessionStorage.setItem('ftSensorSettings', JSON.stringify(settings.ft_sensor));

    try {
        await fetch('/api/settings', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(settings)
        });
        document.getElementById('config-status').textContent = 'Settings applied';
    } catch (e) {
        document.getElementById('config-status').textContent = 'Saved locally';
    }
}

function resetSettings() {
    if (selectedRobot && robotConfigs[selectedRobot]) {
        renderRobotPanel(selectedRobot);
        document.getElementById('config-status').textContent = 'Reset to defaults';
    }
}

// ==================== Utilities ====================

function log(message, level = 'info') {
    const prefix = level === 'error' ? '[ERR]' : '[INFO]';
    console.log(`${prefix} ${message}`);
}

function updateTimestamp() {
    const el = document.getElementById('last-update');
    if (el) el.textContent = new Date().toLocaleTimeString();
}
