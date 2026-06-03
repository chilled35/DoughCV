/**
 * ESPHomeSensor — connects to ESPHome /events SSE stream and fetches initial
 * sensor state via REST.  Bypasses HA's 255-character text sensor state limit.
 */
export class ESPHomeSensor {
  constructor(baseUrl) {
    this._base = baseUrl.replace(/\/$/, '');
    this._source = null;
    this.onConnect    = null;
    this.onDisconnect = null;
    this.onState      = null;  // (id: string, state: string) => void
  }

  connect() {
    if (this._source) this._source.close();
    this._source = new EventSource(this._base + '/events');

    this._source.addEventListener('state', (e) => {
      try {
        const d = JSON.parse(e.data);
        if (d.id && d.state !== undefined && this.onState)
          this.onState(d.id, d.state);
      } catch (err) {
        console.warn('SSE parse error:', err);
      }
    });

    this._source.onopen = () => {
      if (this.onConnect) this.onConnect();
      this._fetchSensors();
    };

    this._source.onerror = () => {
      if (this.onDisconnect) this.onDisconnect();
    };
  }

  disconnect() {
    if (this._source) { this._source.close(); this._source = null; }
  }

  async _fetchSensors() {
    // ESPHome 2026.x uses entity display names in URLs (spaces allowed).
    // Each entry: [url_path, id_passed_to_onState]
    const endpoints = [
      ['/sensor/Rise Height',         'sensor-rise_height'],
      ['/sensor/Footprint Diameter',  'sensor-footprint_diameter'],
      ['/sensor/Detected Dots',       'sensor-detected_dots'],
      ['/binary_sensor/Calibration Valid', 'binary_sensor-calibration_valid'],
    ];
    for (const [ep, id] of endpoints) {
      try {
        const res = await fetch(this._base + ep);
        if (!res.ok) continue;
        const data = await res.json();
        if (data.value !== undefined && this.onState)
          this.onState(id, String(data.value));
      } catch (_) {}
    }
  }
}
