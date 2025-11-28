/**
 * Foxglove Bridge WebSocket Client
 * Handles real-time ROS2 data streaming via Foxglove Bridge
 *
 * Foxglove Bridge Protocol Reference:
 * https://github.com/foxglove/ws-protocol
 */

class FoxgloveClient {
    constructor() {
        this.ws = null;
        this.serverUrl = null;
        this.connected = false;
        this.subscriptions = new Map(); // channelId -> topic
        this.callbacks = new Map(); // topic -> callback function
        this.channels = new Map(); // channelId -> channel info
        this.services = new Map(); // serviceId -> service info
        this.serviceCallbacks = new Map(); // callId -> {resolve, reject}
        this.nextSubscriptionId = 1;
        this.nextCallId = 1;
        this.hasServiceCapability = false;

        // Event handlers
        this.onConnect = null;
        this.onDisconnect = null;
        this.onError = null;
        this.onMessage = null;
        this.onServicesAvailable = null;
    }

    // Binary opcodes (client -> server)
    static CLIENT_OPCODE = {
        SERVICE_CALL_REQUEST: 2
    };

    // Binary opcodes (server -> client)
    static SERVER_OPCODE = {
        MESSAGE_DATA: 1,
        SERVICE_CALL_RESPONSE: 3
    };

    /**
     * Connect to Foxglove Bridge
     * @param {string} host - Server host (e.g., "192.168.1.4")
     * @param {number} port - Server port (default: 8765)
     */
    connect(host, port = 8765) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            console.warn('[Foxglove] Already connected');
            return;
        }

        this.serverUrl = `ws://${host}:${port}`;
        console.log(`[Foxglove] Connecting to ${this.serverUrl}...`);

        try {
            this.ws = new WebSocket(this.serverUrl);
            this.ws.binaryType = 'arraybuffer';

            this.ws.onopen = () => this._handleOpen();
            this.ws.onclose = (event) => this._handleClose(event);
            this.ws.onerror = (error) => this._handleError(error);
            this.ws.onmessage = (event) => this._handleMessage(event);
        } catch (error) {
            console.error('[Foxglove] Connection error:', error);
            if (this.onError) this.onError(error);
        }
    }

    /**
     * Disconnect from Foxglove Bridge
     */
    disconnect() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
        this.connected = false;
        this.subscriptions.clear();
        this.channels.clear();
    }

    /**
     * Subscribe to a ROS2 topic
     * @param {string} topic - Topic name (e.g., "/tf", "/joint_states")
     * @param {function} callback - Callback function for received messages
     */
    subscribe(topic, callback) {
        if (!this.connected) {
            console.warn(`[Foxglove] Not connected, cannot subscribe to ${topic}`);
            return;
        }

        // Find channel by topic name
        let channelId = null;
        for (const [id, channel] of this.channels) {
            if (channel.topic === topic) {
                channelId = id;
                break;
            }
        }

        if (channelId === null) {
            console.warn(`[Foxglove] Topic ${topic} not available`);
            return;
        }

        const subscriptionId = this.nextSubscriptionId++;
        this.subscriptions.set(subscriptionId, { topic, channelId });
        this.callbacks.set(topic, callback);

        // Send subscribe message
        const subscribeMsg = {
            op: 'subscribe',
            subscriptions: [{
                id: subscriptionId,
                channelId: channelId
            }]
        };

        this._send(subscribeMsg);
        console.log(`[Foxglove] Subscribed to ${topic} (channelId: ${channelId})`);
    }

    /**
     * Unsubscribe from a topic
     * @param {string} topic - Topic name
     */
    unsubscribe(topic) {
        let subscriptionId = null;
        for (const [id, sub] of this.subscriptions) {
            if (sub.topic === topic) {
                subscriptionId = id;
                break;
            }
        }

        if (subscriptionId !== null) {
            const unsubscribeMsg = {
                op: 'unsubscribe',
                subscriptionIds: [subscriptionId]
            };
            this._send(unsubscribeMsg);
            this.subscriptions.delete(subscriptionId);
            this.callbacks.delete(topic);
            console.log(`[Foxglove] Unsubscribed from ${topic}`);
        }
    }

    /**
     * Get list of available topics
     * @returns {Array} List of available topics with their types
     */
    getAvailableTopics() {
        const topics = [];
        for (const [id, channel] of this.channels) {
            topics.push({
                id: id,
                topic: channel.topic,
                encoding: channel.encoding,
                schemaName: channel.schemaName
            });
        }
        return topics;
    }

    /**
     * Get list of available services
     * @returns {Array} List of available services
     */
    getAvailableServices() {
        const services = [];
        for (const [id, service] of this.services) {
            services.push({
                id: id,
                name: service.name,
                type: service.type,
                requestSchema: service.requestSchema,
                responseSchema: service.responseSchema
            });
        }
        return services;
    }

    /**
     * Call a ROS2 service
     * @param {string} serviceName - Service name (e.g., "/start_pattern_test")
     * @param {object} request - Request object (will be JSON encoded)
     * @returns {Promise} Promise resolving to the service response
     */
    callService(serviceName, request = {}) {
        return new Promise((resolve, reject) => {
            if (!this.connected) {
                reject(new Error('Not connected to Foxglove Bridge'));
                return;
            }

            if (!this.hasServiceCapability) {
                reject(new Error('Server does not support services'));
                return;
            }

            // Find service by name
            let serviceId = null;
            let serviceInfo = null;
            for (const [id, service] of this.services) {
                if (service.name === serviceName) {
                    serviceId = id;
                    serviceInfo = service;
                    break;
                }
            }

            if (serviceId === null) {
                reject(new Error(`Service ${serviceName} not available`));
                return;
            }

            const callId = this.nextCallId++;
            const encoding = 'json';
            const requestData = JSON.stringify(request);
            const requestBytes = new TextEncoder().encode(requestData);

            // Build binary message
            // Format: [opcode:1][serviceId:4][callId:4][encodingLength:4][encoding:N][request:M]
            const encodingBytes = new TextEncoder().encode(encoding);
            const totalLength = 1 + 4 + 4 + 4 + encodingBytes.length + requestBytes.length;
            const buffer = new ArrayBuffer(totalLength);
            const view = new DataView(buffer);
            const bytes = new Uint8Array(buffer);

            let offset = 0;
            view.setUint8(offset, FoxgloveClient.CLIENT_OPCODE.SERVICE_CALL_REQUEST);
            offset += 1;
            view.setUint32(offset, serviceId, true); // little-endian
            offset += 4;
            view.setUint32(offset, callId, true);
            offset += 4;
            view.setUint32(offset, encodingBytes.length, true);
            offset += 4;
            bytes.set(encodingBytes, offset);
            offset += encodingBytes.length;
            bytes.set(requestBytes, offset);

            // Store callback
            this.serviceCallbacks.set(callId, { resolve, reject, serviceName });

            // Send binary message
            this.ws.send(buffer);
            console.log(`[Foxglove] Service call: ${serviceName} (callId: ${callId})`);

            // Timeout after 30 seconds
            setTimeout(() => {
                if (this.serviceCallbacks.has(callId)) {
                    this.serviceCallbacks.delete(callId);
                    reject(new Error(`Service call timeout: ${serviceName}`));
                }
            }, 30000);
        });
    }

    // ==================== Private Methods ====================

    _handleOpen() {
        console.log('[Foxglove] Connected to', this.serverUrl);
        this.connected = true;
        if (this.onConnect) this.onConnect();
    }

    _handleClose(event) {
        console.log(`[Foxglove] Disconnected (code: ${event.code}, reason: ${event.reason})`);
        this.connected = false;
        this.subscriptions.clear();
        this.channels.clear();
        if (this.onDisconnect) this.onDisconnect(event);
    }

    _handleError(error) {
        console.error('[Foxglove] WebSocket error:', error);
        if (this.onError) this.onError(error);
    }

    _handleMessage(event) {
        try {
            // Foxglove Bridge sends JSON messages
            if (typeof event.data === 'string') {
                const msg = JSON.parse(event.data);
                this._processJsonMessage(msg);
            } else if (event.data instanceof ArrayBuffer) {
                // Binary message (actual topic data)
                this._processBinaryMessage(event.data);
            }
        } catch (error) {
            console.error('[Foxglove] Message parse error:', error);
        }
    }

    _processJsonMessage(msg) {
        switch (msg.op) {
            case 'serverInfo':
                console.log('[Foxglove] Server info:', msg.name, msg.capabilities);
                // Check for services capability
                if (msg.capabilities && msg.capabilities.includes('services')) {
                    this.hasServiceCapability = true;
                    console.log('[Foxglove] Services capability enabled');
                }
                break;

            case 'advertise':
                // Server advertises available channels (topics)
                for (const channel of msg.channels) {
                    this.channels.set(channel.id, {
                        topic: channel.topic,
                        encoding: channel.encoding,
                        schemaName: channel.schemaName,
                        schema: channel.schema
                    });
                }
                console.log(`[Foxglove] ${msg.channels.length} channels available`);
                break;

            case 'unadvertise':
                // Channel no longer available
                for (const channelId of msg.channelIds) {
                    this.channels.delete(channelId);
                }
                break;

            case 'advertiseServices':
                // Server advertises available services
                for (const service of msg.services) {
                    this.services.set(service.id, {
                        name: service.name,
                        type: service.type,
                        requestSchema: service.requestSchema,
                        responseSchema: service.responseSchema
                    });
                }
                console.log(`[Foxglove] ${msg.services.length} services available`);
                if (this.onServicesAvailable) this.onServicesAvailable(this.getAvailableServices());
                break;

            case 'unadvertiseServices':
                // Services no longer available
                for (const serviceId of msg.serviceIds) {
                    this.services.delete(serviceId);
                }
                break;

            case 'serviceCallFailure':
                // Service call failed
                const failCallback = this.serviceCallbacks.get(msg.callId);
                if (failCallback) {
                    this.serviceCallbacks.delete(msg.callId);
                    failCallback.reject(new Error(msg.message));
                    console.error(`[Foxglove] Service call failed: ${msg.message}`);
                }
                break;

            case 'status':
                console.log('[Foxglove] Status:', msg.level, msg.message);
                break;

            default:
                // Pass to general message handler
                if (this.onMessage) this.onMessage(msg);
        }
    }

    _processBinaryMessage(buffer) {
        const view = new DataView(buffer);
        const opcode = view.getUint8(0);

        if (opcode === FoxgloveClient.SERVER_OPCODE.MESSAGE_DATA) {
            // Topic message: [opcode:1][subscriptionId:4][timestamp:8][payload:N]
            const subscriptionId = view.getUint32(1, true);
            const timestampSec = view.getUint32(5, true);
            const timestampNsec = view.getUint32(9, true);
            const payload = buffer.slice(13);

            // Find topic for this subscription
            const sub = this.subscriptions.get(subscriptionId);
            if (sub && this.callbacks.has(sub.topic)) {
                const callback = this.callbacks.get(sub.topic);
                const channel = this.channels.get(sub.channelId);

                // Decode payload based on encoding
                let data;
                if (channel.encoding === 'json') {
                    const decoder = new TextDecoder();
                    data = JSON.parse(decoder.decode(payload));
                } else if (channel.encoding === 'cdr') {
                    // CDR decoding would require additional library
                    // For now, pass raw buffer
                    data = payload;
                } else {
                    data = payload;
                }

                callback({
                    topic: sub.topic,
                    timestamp: timestampSec + timestampNsec / 1e9,
                    data: data
                });
            }
        } else if (opcode === FoxgloveClient.SERVER_OPCODE.SERVICE_CALL_RESPONSE) {
            // Service response: [opcode:1][serviceId:4][callId:4][encodingLength:4][encoding:N][response:M]
            const serviceId = view.getUint32(1, true);
            const callId = view.getUint32(5, true);
            const encodingLength = view.getUint32(9, true);

            let offset = 13;
            const encodingBytes = new Uint8Array(buffer, offset, encodingLength);
            const encoding = new TextDecoder().decode(encodingBytes);
            offset += encodingLength;

            const responsePayload = buffer.slice(offset);

            // Find callback for this call
            const callbackInfo = this.serviceCallbacks.get(callId);
            if (callbackInfo) {
                this.serviceCallbacks.delete(callId);

                // Decode response based on encoding
                let responseData;
                if (encoding === 'json') {
                    const responseText = new TextDecoder().decode(responsePayload);
                    responseData = JSON.parse(responseText);
                } else {
                    responseData = responsePayload;
                }

                console.log(`[Foxglove] Service response received: ${callbackInfo.serviceName}`);
                callbackInfo.resolve(responseData);
            }
        }
    }

    _send(msg) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(msg));
        }
    }
}

/**
 * Helper class for common ROS2 message handling
 */
class ROS2MessageParser {
    /**
     * Parse JointState message
     */
    static parseJointState(data) {
        return {
            header: data.header,
            name: data.name,
            position: data.position,
            velocity: data.velocity,
            effort: data.effort
        };
    }

    /**
     * Parse TFMessage
     */
    static parseTFMessage(data) {
        return data.transforms.map(tf => ({
            header: tf.header,
            childFrameId: tf.child_frame_id,
            translation: tf.transform.translation,
            rotation: tf.transform.rotation
        }));
    }

    /**
     * Format joint angles for display
     */
    static formatJointAngles(jointState, asDegrees = true) {
        if (!jointState || !jointState.position) return 'No data';

        const lines = [];
        for (let i = 0; i < jointState.name.length; i++) {
            const name = jointState.name[i];
            let value = jointState.position[i];
            if (asDegrees) {
                value = (value * 180 / Math.PI).toFixed(2) + '°';
            } else {
                value = value.toFixed(4) + ' rad';
            }
            lines.push(`${name}: ${value}`);
        }
        return lines.join('\n');
    }
}

// Global Foxglove client instance
const foxglove = new FoxgloveClient();
