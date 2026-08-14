// ==========================================================================
// Ranchbot DSP & Sensor Fusion Dedicated Web Worker (dsp-worker.js)
// Offloads Madgwick AHRS, Digital Low-Pass Filters & 64-Point FFT Math
// ==========================================================================

// 6-DOF Madgwick AHRS Implementation
class WorkerMadgwickAHRS {
  constructor(beta = 0.04) {
    this.beta = beta;
    this.q0 = 1.0;
    this.q1 = 0.0;
    this.q2 = 0.0;
    this.q3 = 0.0;
  }

  update(gxDps, gyDps, gzDps, ax, ay, az, dt) {
    const gx = gxDps * (Math.PI / 180.0);
    const gy = gyDps * (Math.PI / 180.0);
    const gz = gzDps * (Math.PI / 180.0);

    let q0 = this.q0, q1 = this.q1, q2 = this.q2, q3 = this.q3;

    let qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    let qDot2 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy);
    let qDot3 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx);
    let qDot4 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx);

    let aLen = Math.sqrt(ax * ax + ay * ay + az * az);
    if (aLen > 0.01) {
      ax /= aLen;
      ay /= aLen;
      az /= aLen;

      const _2q0 = 2.0 * q0, _2q1 = 2.0 * q1, _2q2 = 2.0 * q2, _2q3 = 2.0 * q3;
      const _4q0 = 4.0 * q0, _4q1 = 4.0 * q1, _4q2 = 4.0 * q2;
      const _8q1 = 8.0 * q1, _8q2 = 8.0 * q2;
      const q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

      let s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      let s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      let s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      let s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;

      let sLen = Math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      if (sLen > 0) {
        s0 /= sLen; s1 /= sLen; s2 /= sLen; s3 /= sLen;
        qDot1 -= this.beta * s0;
        qDot2 -= this.beta * s1;
        qDot3 -= this.beta * s2;
        qDot4 -= this.beta * s3;
      }
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    let qNorm = Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (qNorm > 0) {
      this.q0 = q0 / qNorm;
      this.q1 = q1 / qNorm;
      this.q2 = q2 / qNorm;
      this.q3 = q3 / qNorm;
    }
  }

  getEuler() {
    const q0 = this.q0, q1 = this.q1, q2 = this.q2, q3 = this.q3;
    const roll = Math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)) * (180.0 / Math.PI);
    const sinP = 2.0 * (q0 * q2 - q3 * q1);
    const pitch = (Math.abs(sinP) >= 1.0 ? Math.sign(sinP) * (Math.PI / 2) : Math.asin(sinP)) * (180.0 / Math.PI);
    const yaw = ((Math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)) * (180.0 / Math.PI)) % 360 + 360) % 360;
    return { pitch, roll, yaw };
  }

  reset() {
    this.q0 = 1.0; this.q1 = 0.0; this.q2 = 0.0; this.q3 = 0.0;
  }
}

// Digital Low-Pass Filter Implementation
class WorkerDigitalLowPassFilter3Axis {
  constructor(cutoffFreq = 2.0, order = 2) {
    this.cutoffFreq = cutoffFreq;
    this.order = order;
    this.enabled = true;
    this.lastTimestamp = null;
    this.reset();
  }

  reset() {
    this.state = {
      x: { y1: 0, y2: 0, x1: 0, x2: 0, out: 0, initialized: false },
      y: { y1: 0, y2: 0, x1: 0, x2: 0, out: 0, initialized: false },
      z: { y1: 0, y2: 0, x1: 0, x2: 0, out: 0, initialized: false }
    };
    this.lastTimestamp = null;
  }

  setParameters(cutoffFreq, order, enabled) {
    if (cutoffFreq !== undefined && cutoffFreq > 0) this.cutoffFreq = parseFloat(cutoffFreq);
    if (order !== undefined) this.order = parseInt(order, 10);
    if (enabled !== undefined) this.enabled = Boolean(enabled);
  }

  apply(rawX, rawY, rawZ, timestamp = Date.now()) {
    if (!this.enabled) return { x: rawX, y: rawY, z: rawZ };

    let dt = 0.05;
    if (this.lastTimestamp) {
      dt = Math.max(0.001, Math.min(0.5, (timestamp - this.lastTimestamp) / 1000.0));
    }
    this.lastTimestamp = timestamp;

    return {
      x: this._filterChannel(this.state.x, rawX, dt),
      y: this._filterChannel(this.state.y, rawY, dt),
      z: this._filterChannel(this.state.z, rawZ, dt)
    };
  }

  _filterChannel(ch, inputVal, dt) {
    if (!ch.initialized) {
      ch.y1 = inputVal; ch.y2 = inputVal; ch.x1 = inputVal; ch.x2 = inputVal; ch.out = inputVal;
      ch.initialized = true;
      return inputVal;
    }

    if (this.order === 1) {
      const tau = 1.0 / (2.0 * Math.PI * this.cutoffFreq);
      const alpha = dt / (tau + dt);
      ch.out = ch.out + alpha * (inputVal - ch.out);
      return ch.out;
    } else {
      const fc = Math.min(this.cutoffFreq, 0.45 / dt);
      const omega = 2.0 * Math.PI * fc;
      const K = Math.tan((omega * dt) / 2.0);
      const Q = 0.70710678;
      const K2 = K * K;
      const norm = 1.0 + K / Q + K2;

      const b0 = K2 / norm;
      const b1 = 2.0 * b0;
      const b2 = b0;
      const a1 = 2.0 * (K2 - 1.0) / norm;
      const a2 = (1.0 - K / Q + K2) / norm;

      const out = b0 * inputVal + b1 * ch.x1 + b2 * ch.x2 - a1 * ch.y1 - a2 * ch.y2;
      ch.x2 = ch.x1; ch.x1 = inputVal; ch.y2 = ch.y1; ch.y1 = out; ch.out = out;
      return ch.out;
    }
  }
}

// Real-Time 64-Point FFT Implementation
class WorkerFFTAnalyzer {
  constructor(bufferSize = 64) {
    this.bufferSize = bufferSize;
    this.samples = new Float32Array(bufferSize);
    this.index = 0;
    this.sampleRate = 20.0;
  }

  addSample(val) {
    this.samples[this.index] = val;
    this.index = (this.index + 1) % this.bufferSize;
  }

  computeSpectrum() {
    const N = this.bufferSize;
    const real = new Float32Array(N);

    for (let i = 0; i < N; i++) {
      const idx = (this.index + i) % N;
      const w = 0.5 * (1.0 - Math.cos((2.0 * Math.PI * i) / (N - 1)));
      real[i] = this.samples[idx] * w;
    }

    const halfN = N / 2;
    const magnitudes = new Float32Array(halfN);
    let peakMag = 0;
    let peakFreq = 0;
    let sumSq = 0;

    for (let k = 0; k < halfN; k++) {
      let r = 0, im = 0;
      for (let n = 0; n < N; n++) {
        const angle = (2.0 * Math.PI * k * n) / N;
        r += real[n] * Math.cos(angle);
        im -= real[n] * Math.sin(angle);
      }
      const mag = Math.sqrt(r * r + im * im) / N;
      magnitudes[k] = mag;
      sumSq += mag * mag;

      const freq = (k * this.sampleRate) / N;
      if (k > 1 && mag > peakMag) {
        peakMag = mag;
        peakFreq = freq;
      }
    }

    const rms = Math.sqrt(sumSq / halfN);
    return { magnitudes, peakFreq, peakMag, rms };
  }
}

const madgwick = new WorkerMadgwickAHRS(0.04);
const accelFilter = new WorkerDigitalLowPassFilter3Axis(2.0, 2);
const gyroFilter = new WorkerDigitalLowPassFilter3Axis(5.0, 2);
const fft = new WorkerFFTAnalyzer(64);

// Worker Message Handler
self.onmessage = function(e) {
  const { type, data } = e.data;

  if (type === 'PROCESS_IMU') {
    const { calAx, calAy, calAz, calGx, calGy, calGz, dt, timestamp, fusionMode } = data;

    const filtAccel = accelFilter.apply(calAx, calAy, calAz, timestamp);
    const filtGyro = gyroFilter.apply(calGx, calGy, calGz, timestamp);

    const ax = filtAccel.x;
    const ay = filtAccel.y;
    const az = filtAccel.z;
    const gx = filtGyro.x;
    const gy = filtGyro.y;
    const gz = filtGyro.z;

    const totalG = Math.sqrt(ax * ax + ay * ay + az * az);
    fft.addSample(totalG - 1.0);

    let pitch, roll, yaw;
    if (fusionMode === 'madgwick') {
      madgwick.update(gx, gy, gz, ax, ay, az, dt);
      const eul = madgwick.getEuler();
      pitch = eul.pitch;
      roll = eul.roll;
      yaw = eul.yaw;
    } else {
      pitch = Math.atan2(ax, Math.sqrt(ay * ay + az * az)) * (180.0 / Math.PI);
      roll = Math.atan2(ay, Math.sqrt(ax * ax + az * az)) * (180.0 / Math.PI);
      yaw = 0.0;
    }

    const spectrum = fft.computeSpectrum();

    self.postMessage({
      type: 'IMU_PROCESSED',
      data: {
        ax, ay, az,
        gx, gy, gz,
        pitch, roll, yaw,
        totalG,
        spectrum
      }
    });
  } else if (type === 'SET_FILTER_CONFIG') {
    if (data.accel) accelFilter.setParameters(data.accel.cutoff, data.accel.order, data.accel.enabled);
    if (data.gyro) gyroFilter.setParameters(data.gyro.cutoff, data.gyro.order, data.gyro.enabled);
  } else if (type === 'RESET_FUSION') {
    madgwick.reset();
  }
};
