/**
 * ROS2 Interface Config Loader
 * 용도: ros2_interface YAML config 로드 (services, topics, commands)
 * 의존: config-utils.js (deepMerge, loadYaml, loadMergedConfig)
 *
 * Usage:
 *   const config = await Ros2InterfaceLoader.load('lite6');
 *   const serviceName = Ros2InterfaceLoader.getServiceName(config, 'pattern_start');
 */

(function() {
    'use strict';

    const DEFAULT_BASE_URL = '/config/ros2_interface';

    /**
     * Load ROS2 interface configuration (defaults + override merged)
     * @param {string} robotModel - Robot model name (e.g., "lite6", "xarm5")
     * @param {Object} options - Options
     * @param {string} options.baseUrl - Custom base URL
     * @returns {Promise<Object>} Merged config
     */
    async function load(robotModel, options = {}) {
        const { baseUrl = DEFAULT_BASE_URL } = options;

        // Use ConfigUtils if available
        if (typeof ConfigUtils !== 'undefined') {
            return await ConfigUtils.loadMergedConfig(baseUrl, robotModel);
        }

        // Fallback: inline merge
        const defaults = await loadYamlFallback(`${baseUrl}/_defaults.yaml`);
        let override = {};
        try {
            override = await loadYamlFallback(`${baseUrl}/overrides/${robotModel}.yaml`);
        } catch (e) {
            console.warn(`No override for ${robotModel}`);
        }
        return deepMergeFallback(defaults, override);
    }

    // Fallback functions (if config-utils.js not loaded)
    async function loadYamlFallback(url) {
        const resp = await fetch(url);
        if (!resp.ok) throw new Error(`Failed: ${url}`);
        const text = await resp.text();
        return (typeof jsyaml !== 'undefined') ? jsyaml.load(text) : JSON.parse(text);
    }

    function deepMergeFallback(base, override) {
        const result = JSON.parse(JSON.stringify(base));
        for (const key in override) {
            if (override.hasOwnProperty(key)) {
                if (result[key] && typeof result[key] === 'object' && typeof override[key] === 'object'
                    && !Array.isArray(result[key]) && !Array.isArray(override[key])) {
                    result[key] = deepMergeFallback(result[key], override[key]);
                } else {
                    result[key] = JSON.parse(JSON.stringify(override[key]));
                }
            }
        }
        return result;
    }

    // ==================== Getters ====================

    /** Get service name by key */
    function getServiceName(config, key) {
        return config?.services?.[key]?.name || null;
    }

    /** Get service type by key */
    function getServiceType(config, key) {
        return config?.services?.[key]?.type || null;
    }

    /** Get topic name by key */
    function getTopicName(config, key) {
        return config?.topics?.[key]?.name || null;
    }

    /** Get topic type by key */
    function getTopicType(config, key) {
        return config?.topics?.[key]?.type || null;
    }

    /** Get command template by key */
    function getCommandTemplate(config, key) {
        return config?.commands?.[key]?.template || null;
    }

    /**
     * Render command template with robot config values
     * @param {string} template - Command template with {placeholders}
     * @param {Object} robotConfig - Robot config from network_robots
     * @returns {string} Rendered command
     */
    function renderCommand(template, robotConfig) {
        if (!template) return null;
        return template.replace(/\{robot\.(\w+)\}/g, (match, key) => {
            return robotConfig?.robot?.[key] || match;
        });
    }

    // Export
    const Ros2InterfaceLoader = {
        load,
        getServiceName,
        getServiceType,
        getTopicName,
        getTopicType,
        getCommandTemplate,
        renderCommand
    };

    if (typeof window !== 'undefined') {
        window.Ros2InterfaceLoader = Ros2InterfaceLoader;
    }
    if (typeof module !== 'undefined' && module.exports) {
        module.exports = Ros2InterfaceLoader;
    }
})();
