import React, { useState } from 'react';
import { CalibrationConfig } from '../types';

interface CalibrationModalProps {
  config: CalibrationConfig;
  onSave: (config: CalibrationConfig) => void;
  onClose: () => void;
}

export const CalibrationModal: React.FC<CalibrationModalProps> = ({ config, onSave, onClose }) => {
  const [targetSizeMm, setTargetSizeMm] = useState(config.targetSizeMm);
  const [roiX, setRoiX] = useState(config.optimalROI.x);
  const [roiY, setRoiY] = useState(config.optimalROI.y);
  const [roiW, setRoiW] = useState(config.optimalROI.width);
  const [roiH, setRoiH] = useState(config.optimalROI.height);
  const [threshold, setThreshold] = useState(config.laserDetectionThreshold);

  const handleResetDefaults = () => {
    setTargetSizeMm(150);
    setRoiX(50);
    setRoiY(50);
    setRoiW(50);
    setRoiH(50);
    setThreshold(200);
  };

  const handleApply = () => {
    onSave({
      targetSizeMm,
      optimalROI: {
        x: Number(roiX),
        y: Number(roiY),
        width: Number(roiW),
        height: Number(roiH),
      },
      laserDetectionThreshold: Number(threshold),
    });
    onClose();
  };

  const toPct = (mm: number) => (mm / 150) * 100;

  return (
    <div style={{
      position: 'fixed',
      top: 0,
      left: 0,
      right: 0,
      bottom: 0,
      backgroundColor: 'rgba(26, 30, 25, 0.75)',
      backdropFilter: 'blur(3px)',
      zIndex: 100,
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      padding: '20px',
    }}>
      <div className="ref-panel" style={{
        width: '680px',
        maxWidth: '95vw',
        backgroundColor: 'var(--mil-canvas)',
        boxShadow: '0 16px 40px rgba(0, 0, 0, 0.35)',
      }}>
        <div style={{
          backgroundColor: '#272c26',
          color: '#ffffff',
          padding: '12px 20px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
        }}>
          <span style={{ fontFamily: 'var(--font-mono)', fontSize: '12px', fontWeight: 800, letterSpacing: '0.1em' }}>
            CALIBRATION & OPTICAL ROI CONFIGURATION
          </span>
          <button
            onClick={onClose}
            style={{
              background: 'transparent',
              border: 'none',
              color: '#ffffff',
              cursor: 'pointer',
              fontFamily: 'var(--font-mono)',
              fontSize: '16px',
              fontWeight: 800,
            }}
          >
            ✕
          </button>
        </div>

        <div style={{ padding: '24px', display: 'grid', gridTemplateColumns: '220px 1fr', gap: '20px' }}>
          {/* Target Preview */}
          <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
            <div style={{
              width: '200px',
              height: '200px',
              backgroundColor: '#ffffff',
              border: '1px solid #1a1e19',
              position: 'relative',
            }}>
              <svg viewBox="0 0 150 150" style={{ width: '100%', height: '100%', position: 'absolute' }}>
                <circle cx="75" cy="75" r="66" fill="none" stroke="#242923" strokeWidth="0.8" />
                <circle cx="75" cy="75" r="45" fill="none" stroke="#242923" strokeWidth="1.0" />
                <circle cx="75" cy="75" r="25" fill="none" stroke="#1a1e19" strokeWidth="1.2" />
                <circle cx="75" cy="75" r="2.5" fill="#1a1e19" />
              </svg>

              <div style={{
                position: 'absolute',
                left: `${toPct(roiX)}%`,
                top: `${toPct(roiY)}%`,
                width: `${toPct(roiW)}%`,
                height: `${toPct(roiH)}%`,
                border: '2px dashed var(--mil-hit)',
                backgroundColor: 'rgba(44, 89, 52, 0.12)',
                pointerEvents: 'none',
              }} />
            </div>
            <div style={{ fontSize: '10px', fontFamily: 'var(--font-mono)', fontWeight: 700, color: 'var(--text-muted)', marginTop: '8px' }}>
              TARGET: 150 × 150 mm
            </div>
          </div>

          {/* Inputs */}
          <div style={{ display: 'flex', flexDirection: 'column', gap: '12px', fontFamily: 'var(--font-mono)', fontSize: '11px' }}>
            <div>
              <label style={{ color: 'var(--text-dark)', display: 'block', marginBottom: '4px', fontWeight: 700 }}>
                OPTIMAL ZONE ORIGIN X (mm)
              </label>
              <input
                type="number"
                min="0"
                max="140"
                value={roiX}
                onChange={(e) => setRoiX(Number(e.target.value))}
                style={{
                  width: '100%',
                  padding: '7px 10px',
                  backgroundColor: '#ffffff',
                  border: '1px solid var(--mil-border-light)',
                  color: 'var(--text-dark)',
                  fontFamily: 'var(--font-mono)',
                  fontWeight: 700,
                }}
              />
            </div>

            <div>
              <label style={{ color: 'var(--text-dark)', display: 'block', marginBottom: '4px', fontWeight: 700 }}>
                OPTIMAL ZONE ORIGIN Y (mm)
              </label>
              <input
                type="number"
                min="0"
                max="140"
                value={roiY}
                onChange={(e) => setRoiY(Number(e.target.value))}
                style={{
                  width: '100%',
                  padding: '7px 10px',
                  backgroundColor: '#ffffff',
                  border: '1px solid var(--mil-border-light)',
                  color: 'var(--text-dark)',
                  fontFamily: 'var(--font-mono)',
                  fontWeight: 700,
                }}
              />
            </div>

            <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '10px' }}>
              <div>
                <label style={{ color: 'var(--text-dark)', display: 'block', marginBottom: '4px', fontWeight: 700 }}>
                  WIDTH (mm)
                </label>
                <input
                  type="number"
                  min="10"
                  max="150"
                  value={roiW}
                  onChange={(e) => setRoiW(Number(e.target.value))}
                  style={{
                    width: '100%',
                    padding: '7px 10px',
                    backgroundColor: '#ffffff',
                    border: '1px solid var(--mil-border-light)',
                    color: 'var(--text-dark)',
                    fontFamily: 'var(--font-mono)',
                    fontWeight: 700,
                  }}
                />
              </div>

              <div>
                <label style={{ color: 'var(--text-dark)', display: 'block', marginBottom: '4px', fontWeight: 700 }}>
                  HEIGHT (mm)
                </label>
                <input
                  type="number"
                  min="10"
                  max="150"
                  value={roiH}
                  onChange={(e) => setRoiH(Number(e.target.value))}
                  style={{
                    width: '100%',
                    padding: '7px 10px',
                    backgroundColor: '#ffffff',
                    border: '1px solid var(--mil-border-light)',
                    color: 'var(--text-dark)',
                    fontFamily: 'var(--font-mono)',
                    fontWeight: 700,
                  }}
                />
              </div>
            </div>

            <div>
              <label style={{ color: 'var(--text-dark)', display: 'block', marginBottom: '4px', fontWeight: 700 }}>
                OV5647 SENSOR DETECTION THRESHOLD (0–255)
              </label>
              <input
                type="range"
                min="50"
                max="255"
                value={threshold}
                onChange={(e) => setThreshold(Number(e.target.value))}
                style={{ width: '100%', accentColor: 'var(--mil-dark)' }}
              />
              <div style={{ display: 'flex', justifyContent: 'space-between', color: 'var(--text-muted)', fontSize: '10px', fontWeight: 600 }}>
                <span>50</span>
                <span style={{ color: 'var(--text-dark)', fontWeight: 800 }}>{threshold}</span>
                <span>255</span>
              </div>
            </div>
          </div>
        </div>

        <div style={{
          padding: '14px 20px',
          backgroundColor: '#ffffff',
          borderTop: '1px solid var(--mil-border-light)',
          display: 'flex',
          justifyContent: 'space-between',
        }}>
          <button onClick={handleResetDefaults} className="ref-btn ref-btn-light">
            RESET DEFAULTS
          </button>
          <div style={{ display: 'flex', gap: '8px' }}>
            <button onClick={onClose} className="ref-btn ref-btn-light">
              CANCEL
            </button>
            <button onClick={handleApply} className="ref-btn">
              SAVE
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};
