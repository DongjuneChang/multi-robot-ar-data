/**
 * Monitoring Panel - app.js
 * Live View for Map, Tracking, Camera Vision
 */

// State
let mapData = {};
let trackingData = {};
let gridVisible = true;
let socket = null;

// Initialize
document.addEventListener('DOMContentLoaded', function() {
    initTabs();
    initButtons();
    loadMapData();
    loadTrackingData();
    initCameraGrid();
    initSocketIO();
    addLog('Monitoring panel initialized');
});

// Tab handling
function initTabs() {
    document.querySelectorAll('.tab').forEach(tab => {
        tab.addEventListener('click', function() {
            const tabId = this.dataset.tab;

            // Update tab buttons
            document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
            this.classList.add('active');

            // Update tab content
            document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));
            document.getElementById(tabId).classList.add('active');

            // Redraw if map tab
            if (tabId === 'map') {
                drawMap();
            }
        });
    });
}

// Button handlers
function initButtons() {
    document.getElementById('btn-center-map')?.addEventListener('click', centerMap);
    document.getElementById('btn-refresh-map')?.addEventListener('click', loadMapData);
    document.getElementById('btn-toggle-grid')?.addEventListener('click', toggleGrid);
    document.getElementById('btn-refresh-tracking')?.addEventListener('click', loadTrackingData);
    document.getElementById('btn-scan-cameras')?.addEventListener('click', initCameraGrid);
    document.getElementById('btn-clear-log')?.addEventListener('click', clearLog);
}

// Load map data
async function loadMapData() {
    try {
        const response = await fetch('/api/map_data');
        mapData = await response.json();
        drawMap();
        updateQRTable();
        updateUserTable();
        addLog('Map data loaded');
    } catch (error) {
        console.error('Failed to load map data:', error);
        addLog('Failed to load map data', 'error');
    }
}

// Load tracking data
async function loadTrackingData() {
    try {
        const response = await fetch('/api/tracking_data');
        trackingData = await response.json();
        updateTrackingTable();
        updateRobotTable();
        document.getElementById('tracking-last-update').textContent = new Date().toLocaleTimeString();
        addLog('Tracking data refreshed');
    } catch (error) {
        console.error('Failed to load tracking data:', error);
        addLog('Failed to load tracking data', 'error');
    }
}

// Draw map with markers
function drawMap() {
    const container = document.getElementById('map-container');
    if (!container) return;

    // Remove existing markers (keep grid and canvas)
    container.querySelectorAll('.map-marker').forEach(m => m.remove());

    // Get container dimensions
    const rect = container.getBoundingClientRect();
    const centerX = rect.width / 2;
    const centerY = rect.height / 2;
    const scale = 100; // pixels per meter

    // Draw QR markers
    if (mapData.registered_qr_codes) {
        Object.entries(mapData.registered_qr_codes).forEach(([qrId, qr]) => {
            const marker = document.createElement('div');
            marker.className = 'map-marker qr-marker';
            marker.textContent = qrId;

            // Position (convert from map coordinates)
            const pos = qr.map_position || [0, 0];
            marker.style.left = (centerX + pos[0] * scale) + 'px';
            marker.style.top = (centerY - pos[1] * scale) + 'px';

            marker.title = `${qrId}\nRegistered by: ${qr.registered_by || 'Unknown'}`;
            container.appendChild(marker);
        });
    }

    // Draw user markers
    if (trackingData.active_users) {
        Object.entries(trackingData.active_users).forEach(([userId, user]) => {
            const marker = document.createElement('div');
            marker.className = 'map-marker user-marker';

            if (userId === 'dongjune_chang') marker.classList.add('admin');
            if (!user.online) marker.classList.add('offline');

            const pos = user.realtime_pose?.map_position || [0, 0];
            marker.style.left = (centerX + pos[0] * scale) + 'px';
            marker.style.top = (centerY - pos[1] * scale) + 'px';

            const userName = mapData.registered_users?.[userId]?.user_name || userId;
            marker.title = `${userName}\nStatus: ${user.online ? 'Online' : 'Offline'}`;
            container.appendChild(marker);
        });
    }

    // Draw robot markers
    if (trackingData.robot_telemetry) {
        Object.entries(trackingData.robot_telemetry).forEach(([robotId, robot]) => {
            const marker = document.createElement('div');
            marker.className = 'map-marker robot-marker';

            const pos = robot.map_position || [0, 0];
            marker.style.left = (centerX + pos[0] * scale) + 'px';
            marker.style.top = (centerY - pos[1] * scale) + 'px';

            marker.title = `${robotId}\nBattery: ${robot.battery || 0}%`;
            container.appendChild(marker);
        });
    }
}

// Center map
function centerMap() {
    const container = document.getElementById('map-container');
    if (container) {
        container.scrollTo({
            left: (container.scrollWidth - container.clientWidth) / 2,
            top: (container.scrollHeight - container.clientHeight) / 2,
            behavior: 'smooth'
        });
    }
    addLog('Map centered');
}

// Toggle grid
function toggleGrid() {
    const grid = document.getElementById('map-grid');
    if (grid) {
        grid.classList.toggle('hidden');
        gridVisible = !grid.classList.contains('hidden');
        addLog(`Grid ${gridVisible ? 'shown' : 'hidden'}`);
    }
}

// Update QR table
function updateQRTable() {
    const tbody = document.getElementById('qr-table-body');
    if (!tbody) return;

    if (!mapData.registered_qr_codes || Object.keys(mapData.registered_qr_codes).length === 0) {
        tbody.innerHTML = '<tr><td colspan="4">No QR codes registered</td></tr>';
        return;
    }

    tbody.innerHTML = Object.entries(mapData.registered_qr_codes).map(([qrId, qr]) => `
        <tr>
            <td>${qrId}</td>
            <td>${qr.registered_by || '-'}</td>
            <td>(${(qr.map_position || [0, 0]).map(v => v.toFixed(2)).join(', ')})</td>
            <td><span class="badge badge-active">Active</span></td>
        </tr>
    `).join('');
}

// Update user table
function updateUserTable() {
    const tbody = document.getElementById('user-table-body');
    if (!tbody) return;

    if (!mapData.registered_users || Object.keys(mapData.registered_users).length === 0) {
        tbody.innerHTML = '<tr><td colspan="5">No users registered</td></tr>';
        return;
    }

    tbody.innerHTML = Object.entries(mapData.registered_users).map(([userId, user]) => {
        const isOnline = trackingData.active_users?.[userId]?.online;
        return `
            <tr>
                <td>${user.user_name || userId}</td>
                <td>${user.device_id || '-'}</td>
                <td>${user.role || 'User'}</td>
                <td>${(user.accessible_qr_codes || []).join(', ') || '-'}</td>
                <td><span class="badge ${isOnline ? 'badge-online' : 'badge-offline'}">${isOnline ? 'Online' : 'Offline'}</span></td>
            </tr>
        `;
    }).join('');
}

// Update tracking table
function updateTrackingTable() {
    const tbody = document.getElementById('tracking-user-table');
    if (!tbody) return;

    if (!trackingData.active_users || Object.keys(trackingData.active_users).length === 0) {
        tbody.innerHTML = '<tr><td colspan="6">No active users</td></tr>';
        return;
    }

    tbody.innerHTML = Object.entries(trackingData.active_users)
        .filter(([_, user]) => user.online)
        .map(([userId, user]) => {
            const pose = user.realtime_pose || {};
            const pos = pose.map_position || [0, 0];
            const userName = mapData.registered_users?.[userId]?.user_name || userId;
            return `
                <tr>
                    <td>${userName}</td>
                    <td>(${pos.map(v => v.toFixed(2)).join(', ')})</td>
                    <td>${(pose.height || 1.6).toFixed(2)}m</td>
                    <td>${user.activity || 'Idle'}</td>
                    <td>${(user.active_robots || []).join(', ') || '-'}</td>
                    <td>${user.last_seen || '-'}</td>
                </tr>
            `;
        }).join('');
}

// Update robot table
function updateRobotTable() {
    const tbody = document.getElementById('robot-table-body');
    if (!tbody) return;

    if (!trackingData.robot_telemetry || Object.keys(trackingData.robot_telemetry).length === 0) {
        tbody.innerHTML = '<tr><td colspan="6">No robots active</td></tr>';
        return;
    }

    tbody.innerHTML = Object.entries(trackingData.robot_telemetry).map(([robotId, robot]) => {
        const pos = robot.map_position || [0, 0];
        const status = robot.status || 'idle';
        return `
            <tr>
                <td>${robotId}</td>
                <td>${robot.owner || '-'}</td>
                <td>(${pos.map(v => v.toFixed(2)).join(', ')})</td>
                <td>${robot.battery || 0}%</td>
                <td><span class="badge badge-${status === 'active' || status === 'moving' ? 'active' : 'idle'}">${status}</span></td>
                <td>${robot.task || '-'}</td>
            </tr>
        `;
    }).join('');
}

// Initialize camera grid
async function initCameraGrid() {
    const grid = document.getElementById('camera-grid');
    if (!grid) return;

    // Try to load users for camera cells
    let users = {};
    try {
        const response = await fetch('/api/users');
        const data = await response.json();
        users = data.users || {};
    } catch (error) {
        console.log('Could not load users for camera grid');
    }

    // Create 4 camera cells (2x2 grid)
    const userEntries = Object.entries(users);
    const maxSlots = 4;

    let html = '';
    for (let i = 0; i < maxSlots; i++) {
        if (i < userEntries.length) {
            const [userId, user] = userEntries[i];
            html += `
                <div class="camera-cell">
                    <div class="camera-header">
                        <div>
                            <h4>${user.user_name || userId}</h4>
                            <span class="device-info">${user.device_id || 'Unknown'} (${user.device_ip || '-'})</span>
                        </div>
                        <span class="status-dot disconnected"></span>
                    </div>
                    <div class="camera-frame">
                        <div class="placeholder">Click to connect</div>
                    </div>
                </div>
            `;
        } else {
            html += `
                <div class="camera-cell">
                    <div class="camera-header">
                        <div>
                            <h4>Available Slot</h4>
                            <span class="device-info">No Device</span>
                        </div>
                        <span class="status-dot disconnected"></span>
                    </div>
                    <div class="camera-frame">
                        <div class="placeholder">No device assigned</div>
                    </div>
                </div>
            `;
        }
    }

    grid.innerHTML = html;
    addLog('Camera grid initialized');
}

// Socket.IO for real-time updates
function initSocketIO() {
    // Check if Socket.IO is available
    if (typeof io === 'undefined') {
        console.log('Socket.IO not available, using polling');
        setInterval(loadTrackingData, 5000);
        return;
    }

    try {
        socket = io();

        socket.on('connected', (data) => {
            addLog('Connected to server (WebSocket)');
        });

        socket.on('tracking_update', (data) => {
            trackingData = data;
            drawMap();
            updateTrackingTable();
            updateRobotTable();
            document.getElementById('tracking-last-update').textContent = new Date().toLocaleTimeString();
        });

        socket.on('disconnect', () => {
            addLog('Disconnected from server', 'warning');
        });
    } catch (error) {
        console.log('WebSocket connection failed, using polling');
        setInterval(loadTrackingData, 5000);
    }
}

// Log functions
function addLog(message, type = '') {
    const log = document.getElementById('log-content');
    if (!log) return;

    const time = new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    entry.textContent = `[${time}] ${message}`;
    log.appendChild(entry);
    log.scrollTop = log.scrollHeight;
}

function clearLog() {
    const log = document.getElementById('log-content');
    if (log) {
        log.innerHTML = '<div class="log-entry">[System] Log cleared</div>';
    }
}
