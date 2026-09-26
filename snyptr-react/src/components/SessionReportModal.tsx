import React from 'react';
import { OptimalROI, SessionReport } from '../types';

interface SessionReportModalProps {
  report: SessionReport;
  optimalROI: OptimalROI;
  onStartNewSession: () => void;
  onClose: () => void;
}

export const SessionReportModal: React.FC<SessionReportModalProps> = ({
  report,
  optimalROI,
  onStartNewSession,
  onClose,
}) => {
  const toPct = (mm: number) => (mm / 150) * 100;

  const handleExportJson = () => {
    const dataStr = 'data:text/json;charset=utf-8,' + encodeURIComponent(JSON.stringify(report, null, 2));
    const downloadAnchor = document.createElement('a');
    downloadAnchor.setAttribute('href', dataStr);
    downloadAnchor.setAttribute('download', `SNYPTR_SESSION_${report.sessionId}.json`);
    document.body.appendChild(downloadAnchor);
    downloadAnchor.click();
    downloadAnchor.remove();
  };

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
      padding: '24px',
    }}>
      <div className="ref-panel" style={{
        width: '980px',
        maxWidth: '95vw',
        maxHeight: '92vh',
        display: 'flex',
        flexDirection: 'column',
        boxShadow: '0 16px 40px rgba(0, 0, 0, 0.4)',
        backgroundColor: 'var(--mil-canvas)',
      }}>
        {/* Header */}
        <div style={{
          backgroundColor: '#272c26',
          color: '#ffffff',
          padding: '12px 20px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
        }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
            <span style={{ fontSize: '14px' }}>⊕</span>
            <span style={{ fontFamily: 'var(--font-mono)', fontSize: '13px', fontWeight: 800, letterSpacing: '0.1em' }}>
              20-SHOT DRILL COMPLETE // RANGE REPORT
            </span>
          </div>
          <div style={{ display: 'flex', alignItems: 'center', gap: '12px' }}>
            <span style={{ fontSize: '11px', color: 'var(--text-sand)', fontFamily: 'var(--font-mono)' }}>
              SESSION: {report.sessionId}
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
        </div>

        {/* Content */}
        <div style={{
          padding: '20px 24px',
          overflowY: 'auto',
          display: 'flex',
          flexDirection: 'column',
          gap: '16px',
        }}>
          {/* Top Row: 4 Metric Cards */}
          <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '12px' }}>
            <div style={{ backgroundColor: '#ffffff', padding: '14px', border: '1px solid var(--mil-border-light)' }}>
              <div style={{ fontSize: '10px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700 }}>TOTAL SHOTS</div>
              <div style={{ fontFamily: 'var(--font-mono)', fontSize: '28px', fontWeight: 900, color: 'var(--text-dark)', marginTop: '2px' }}>
                {report.totalShots}
              </div>
            </div>

            <div style={{ backgroundColor: '#ffffff', padding: '14px', border: '1px solid var(--mil-border-light)', borderLeft: '4px solid var(--mil-hit)' }}>
              <div style={{ fontSize: '10px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700 }}>QUALIFIED HITS</div>
              <div style={{ fontFamily: 'var(--font-mono)', fontSize: '28px', fontWeight: 900, color: 'var(--mil-hit)', marginTop: '2px' }}>
                {report.hits}
              </div>
            </div>

            <div style={{ backgroundColor: '#ffffff', padding: '14px', border: '1px solid var(--mil-border-light)', borderLeft: '4px solid var(--mil-miss)' }}>
              <div style={{ fontSize: '10px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700 }}>ZONE MISSES</div>
              <div style={{ fontFamily: 'var(--font-mono)', fontSize: '28px', fontWeight: 900, color: 'var(--mil-miss)', marginTop: '2px' }}>
                {report.misses}
              </div>
            </div>

            <div style={{ backgroundColor: '#ffffff', padding: '14px', border: '1px solid var(--mil-border-light)', borderLeft: '4px solid var(--mil-dark)' }}>
              <div style={{ fontSize: '10px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700 }}>FINAL ACCURACY</div>
              <div style={{ fontFamily: 'var(--font-mono)', fontSize: '28px', fontWeight: 900, color: 'var(--text-dark)', marginTop: '2px' }}>
                {report.accuracy.toFixed(1)}%
              </div>
            </div>
          </div>

          {/* Middle Row: Shot Map + Distance Analysis */}
          <div style={{ display: 'grid', gridTemplateColumns: '300px 1fr', gap: '16px' }}>
            {/* Target Shot Map */}
            <div style={{
              backgroundColor: '#ffffff',
              border: '1px solid var(--mil-border-light)',
              padding: '14px',
              display: 'flex',
              flexDirection: 'column',
              alignItems: 'center',
            }}>
              <div style={{
                fontFamily: 'var(--font-mono)',
                fontSize: '11px',
                fontWeight: 800,
                color: 'var(--text-dark)',
                marginBottom: '10px',
                alignSelf: 'flex-start',
              }}>
                TARGET SHOT DISPERSION MAP
              </div>

              <div style={{
                width: '250px',
                height: '250px',
                backgroundColor: '#ffffff',
                border: '1px solid #1a1e19',
                position: 'relative',
              }}>
                <svg viewBox="0 0 150 150" style={{ width: '100%', height: '100%', position: 'absolute' }}>
                  <line x1="75" y1="0" x2="75" y2="150" stroke="#b0a99c" strokeWidth="0.5" strokeDasharray="1 3" />
                  <line x1="0" y1="75" x2="150" y2="75" stroke="#b0a99c" strokeWidth="0.5" strokeDasharray="1 3" />
                  <circle cx="75" cy="75" r="66" fill="none" stroke="#242923" strokeWidth="0.8" />
                  <circle cx="75" cy="75" r="45" fill="none" stroke="#242923" strokeWidth="1.0" />
                  <circle cx="75" cy="75" r="25" fill="none" stroke="#1a1e19" strokeWidth="1.2" />

                  <rect
                    x={optimalROI.x}
                    y={optimalROI.y}
                    width={optimalROI.width}
                    height={optimalROI.height}
                    fill="rgba(44, 89, 52, 0.08)"
                    stroke="#2c5934"
                    strokeWidth="1.6"
                    strokeDasharray="4 2"
                  />
                  <circle cx="75" cy="75" r="2.5" fill="#1a1e19" />
                </svg>

                {/* Plot shots */}
                {report.shots.map((s) => (
                  <div
                    key={s.id}
                    style={{
                      position: 'absolute',
                      left: `${toPct(s.xMm)}%`,
                      top: `${toPct(s.yMm)}%`,
                      width: '8px',
                      height: '8px',
                      transform: 'translate(-50%, -50%)',
                      borderRadius: '50%',
                      backgroundColor: s.result === 'HIT' ? 'var(--mil-hit)' : 'var(--mil-miss)',
                      border: '1.5px solid #ffffff',
                    }}
                    title={`Shot #${s.shotNumber}: ${s.result}`}
                  />
                ))}
              </div>

              <div style={{
                display: 'flex',
                gap: '14px',
                marginTop: '10px',
                fontSize: '10px',
                fontFamily: 'var(--font-mono)',
                fontWeight: 700,
                color: 'var(--text-dark)',
              }}>
                <span>● HIT ({report.hits})</span>
                <span>● MISS ({report.misses})</span>
                <span>□ OPTIMAL ROI</span>
              </div>
            </div>

            {/* Right: Distance Breakdown & Miss Analysis */}
            <div style={{ display: 'flex', flexDirection: 'column', gap: '14px' }}>
              <div style={{ backgroundColor: '#ffffff', border: '1px solid var(--mil-border-light)', padding: '14px' }}>
                <div style={{ fontFamily: 'var(--font-mono)', fontSize: '11px', fontWeight: 800, color: 'var(--text-dark)', marginBottom: '8px' }}>
                  ACCURACY BY DISTANCE BRACKET
                </div>

                <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: '8px' }}>
                  {report.distanceBands.map((band) => (
                    <div key={band.band} style={{
                      backgroundColor: 'var(--mil-card-bg)',
                      border: '1px solid var(--mil-border-light)',
                      padding: '10px 8px',
                      fontSize: '11px',
                      fontFamily: 'var(--font-mono)',
                    }}>
                      <div style={{ color: 'var(--text-muted)', fontSize: '10px', fontWeight: 700 }}>{band.band}</div>
                      <div style={{
                        fontSize: '18px',
                        fontWeight: 900,
                        color: 'var(--text-dark)',
                        marginTop: '4px',
                      }}>
                        {band.accuracy !== null ? `${band.accuracy.toFixed(0)}%` : 'INSUFFICIENT'}
                      </div>
                      <div style={{ color: 'var(--text-muted)', fontSize: '9px', marginTop: '2px' }}>
                        {band.hits} H / {band.totalShots} SHOTS
                      </div>
                    </div>
                  ))}
                </div>
              </div>

              <div style={{ backgroundColor: '#ffffff', border: '1px solid var(--mil-border-light)', padding: '14px' }}>
                <div style={{ fontFamily: 'var(--font-mono)', fontSize: '11px', fontWeight: 800, color: 'var(--text-dark)', marginBottom: '8px' }}>
                  MISS CLUSTER DIAGNOSTICS
                </div>
                <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '10px', fontSize: '11px', fontFamily: 'var(--font-mono)' }}>
                  <div style={{ backgroundColor: 'var(--mil-card-bg)', padding: '10px', border: '1px solid var(--mil-border-light)' }}>
                    <div style={{ color: 'var(--text-muted)', fontSize: '10px', fontWeight: 700 }}>PRIMARY MISS BIAS</div>
                    <div style={{ fontSize: '13px', fontWeight: 800, color: 'var(--mil-miss)', marginTop: '3px' }}>
                      {report.missDistribution.dominantDirection
                        ? `${report.missDistribution.dominantDirection} OF OPTIMAL`
                        : 'NONE DETECTED'}
                    </div>
                  </div>

                  <div style={{ backgroundColor: 'var(--mil-card-bg)', padding: '10px', border: '1px solid var(--mil-border-light)' }}>
                    <div style={{ color: 'var(--text-muted)', fontSize: '10px', fontWeight: 700 }}>HIGHEST MISS BAND</div>
                    <div style={{ fontSize: '13px', fontWeight: 800, color: 'var(--mil-miss)', marginTop: '3px' }}>
                      {report.missDistribution.highestMissBand
                        ? `${report.missDistribution.highestMissBand} BRACKET`
                        : 'EVENLY DISTRIBUTED'}
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Bottom Row: Training Focus */}
          <div style={{
            display: 'grid',
            gridTemplateColumns: '1.2fr 1fr',
            gap: '14px',
            backgroundColor: '#ffffff',
            border: '1px solid var(--mil-border-light)',
            padding: '14px 18px',
          }}>
            <div>
              <div style={{ fontFamily: 'var(--font-mono)', fontSize: '11px', fontWeight: 800, color: 'var(--text-dark)', marginBottom: '6px' }}>
                DATA-BACKED OBSERVATIONS
              </div>
              <ul style={{ paddingLeft: '18px', fontSize: '12px', color: 'var(--text-dark)', display: 'flex', flexDirection: 'column', gap: '4px' }}>
                {report.observations.map((obs, i) => (
                  <li key={i}>{obs}</li>
                ))}
              </ul>
            </div>

            <div>
              <div style={{ fontFamily: 'var(--font-mono)', fontSize: '11px', fontWeight: 800, color: 'var(--text-dark)', marginBottom: '6px' }}>
                TRAINING FOCUS
              </div>
              <ol style={{ paddingLeft: '18px', fontSize: '12px', color: 'var(--text-dark)', display: 'flex', flexDirection: 'column', gap: '4px' }}>
                {report.trainingFocus.map((foc, i) => (
                  <li key={i}>{foc}</li>
                ))}
              </ol>
            </div>
          </div>
        </div>

        {/* Footer */}
        <div style={{
          padding: '12px 24px',
          backgroundColor: '#272c26',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
        }}>
          <button
            onClick={handleExportJson}
            className="ref-btn ref-btn-light"
          >
            EXPORT JSON
          </button>

          <div style={{ display: 'flex', gap: '10px' }}>
            <button
              onClick={onClose}
              className="ref-btn ref-btn-light"
            >
              CLOSE
            </button>
            <button
              onClick={onStartNewSession}
              className="ref-btn"
              style={{ backgroundColor: '#ffffff', color: 'var(--text-dark)' }}
            >
              START NEW SESSION
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};
