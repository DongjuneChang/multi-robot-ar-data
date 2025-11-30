/**
 * AI User Preferences Loader
 * ai/ai_user_preferences/ config 로드
 *
 * 사용법:
 *   const prefs = await AIUserPreferencesLoader.loadUserPreferences('dongjune_chang');
 *   console.log(prefs.personas, prefs.default_mimic);
 */

const AIUserPreferencesLoader = {
    CONFIG_BASE_URL: '/config/ai/ai_user_preferences',
    _defaults: null,
    _cache: {},

    /**
     * defaults 로드 (캐싱)
     */
    async loadDefaults() {
        if (this._defaults) return this._defaults;

        try {
            this._defaults = await ConfigUtils.loadYaml(`${this.CONFIG_BASE_URL}/_defaults.yaml`);
            console.log('[AIUserPreferencesLoader] Loaded defaults');
            return this._defaults;
        } catch (e) {
            console.error('[AIUserPreferencesLoader] Failed to load defaults:', e);
            return this._getHardcodedDefaults();
        }
    },

    /**
     * 사용자별 AI 선호 설정 로드
     * @param {string} userId - user_id (e.g., 'dongjune_chang')
     * @returns {Promise<Object>} Merged preferences
     */
    async loadUserPreferences(userId) {
        // 캐시 확인
        if (this._cache[userId]) {
            return this._cache[userId];
        }

        const defaults = await this.loadDefaults();

        // override 로드 시도
        let override = {};
        try {
            override = await ConfigUtils.loadYaml(`${this.CONFIG_BASE_URL}/overrides/${userId}.yaml`);
            console.log(`[AIUserPreferencesLoader] Loaded override for ${userId}`);
        } catch (e) {
            console.warn(`[AIUserPreferencesLoader] No override for ${userId}, using defaults`);
        }

        // 머지
        const merged = ConfigUtils.deepMerge(defaults, override);
        this._cache[userId] = merged;

        return merged;
    },

    /**
     * 여러 사용자의 AI 선호 설정 로드
     * @param {string[]} userIds - user_id 배열
     * @returns {Promise<Object>} { userId: preferences, ... }
     */
    async loadMultipleUserPreferences(userIds) {
        const result = {};
        for (const userId of userIds) {
            result[userId] = await this.loadUserPreferences(userId);
        }
        return result;
    },

    /**
     * 사용자가 사용할 수 있는 personas 목록 가져오기
     * @param {string} userId
     * @returns {Promise<string[]>}
     */
    async getUserPersonas(userId) {
        const prefs = await this.loadUserPreferences(userId);
        return prefs.personas || ['basic'];
    },

    /**
     * 사용자의 default mimic expert 가져오기
     * @param {string} userId
     * @returns {Promise<string|null>}
     */
    async getUserDefaultMimic(userId) {
        const prefs = await this.loadUserPreferences(userId);
        return prefs.default_mimic || null;
    },

    /**
     * 캐시 초기화
     */
    clearCache() {
        this._defaults = null;
        this._cache = {};
    },

    /**
     * Fallback defaults
     */
    _getHardcodedDefaults() {
        return {
            personas: ['basic'],
            default_mimic: null,
            rag_sources: [],
            crew_agents: []
        };
    }
};

// Export for browser global
if (typeof window !== 'undefined') {
    window.AIUserPreferencesLoader = AIUserPreferencesLoader;
}
