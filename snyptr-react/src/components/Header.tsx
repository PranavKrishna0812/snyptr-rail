import React from 'react';

interface HeaderProps {
  sessionNumber: number;
  systemStatus: string;
  isEmergencyStop: boolean;
  onOpenCalibration?: () => void;
  onResetEmergency?: () => void;
  onStartNewSession?: () => void;
}

export const Header: React.FC<HeaderProps> = ({
  sessionNumber,
  systemStatus,
  isEmergencyStop,
  onOpenCalibration,
  onResetEmergency,
  onStartNewSession,
}) => {
  return (
    <div style={{ userSelect: 'none', display: 'flex', flexDirection: 'column' }}>
      {/* Top Banner (Reference Image Header Style) */}
      <header style={{
        backgroundColor: '#272c26',
        padding: '10px 24px',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        borderBottom: '1px solid #1a1e19',
      }}>
        {/* Left: Boxed Logo Badge (Matching [💀 MILITARY] from reference) */}
        <div style={{ display: 'flex', alignItems: 'center', gap: '16px' }}>
          <div style={{
            border: '2px solid #ffffff',
            padding: '4px 12px',
            display: 'flex',
            alignItems: 'center',
            gap: '8px',
            backgroundColor: '#1d221c',
          }}>
            <span style={{ fontSize: '14px', color: '#ffffff' }}>⊕</span>
            <span style={{
              fontFamily: 'var(--font-mono)',
              fontSize: '16px',
              fontWeight: 900,
              letterSpacing: '0.14em',
              color: '#ffffff',
            }}>
              SNYPTR
            </span>
          </div>

          <div style={{
            fontFamily: 'var(--font-sans)',
            fontSize: '11px',
            fontWeight: 700,
            letterSpacing: '0.1em',
            color: 'var(--text-sand)',
            textTransform: 'uppercase',
          }}>
            2-Metre Moving Target System
          </div>
        </div>

        {/* Right: Clean Controls */}
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          {isEmergencyStop ? (
            <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
              <span style={{
                backgroundColor: 'var(--mil-miss)',
                color: '#ffffff',
                padding: '4px 10px',
                fontFamily: 'var(--font-mono)',
                fontSize: '11px',
                fontWeight: 800,
              }}>
                ESTOP ACTIVE
              </span>
              {onResetEmergency && (
                <button
                  onClick={onResetEmergency}
                  className="ref-btn"
                  style={{ backgroundColor: '#ffffff', color: 'var(--text-dark)' }}
                >
                  RESET [ESC]
                </button>
              )}
            </div>
          ) : (
            <span style={{
              backgroundColor: '#353c34',
              color: '#ffffff',
              padding: '4px 10px',
              fontFamily: 'var(--font-mono)',
              fontSize: '11px',
              fontWeight: 700,
            }}>
              SYS: {systemStatus}
            </span>
          )}

          <span style={{
            backgroundColor: '#1d221c',
            border: '1px solid #3a4239',
            color: 'var(--text-sand)',
            padding: '4px 10px',
            fontFamily: 'var(--font-mono)',
            fontSize: '11px',
            fontWeight: 700,
          }}>
            SESSION {String(sessionNumber).padStart(2, '0')}
          </span>

          {onStartNewSession && (
            <button
              onClick={onStartNewSession}
              className="ref-btn"
            >
              NEW SESSION
            </button>
          )}

          {onOpenCalibration && (
            <button
              onClick={onOpenCalibration}
              className="ref-btn ref-btn-light"
              title="Target Calibration"
            >
              CALIBRATE
            </button>
          )}
        </div>
      </header>

      {/* Sub-Bar (Matching Navigation Strip from reference) */}
      <div style={{
        backgroundColor: '#1e221d',
        padding: '6px 24px',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        fontSize: '11px',
        fontFamily: 'var(--font-mono)',
        color: 'var(--text-sand)',
        borderBottom: '1px solid var(--mil-border-light)',
      }}>
        <div style={{ display: 'flex', gap: '20px', fontWeight: 600 }}>
          <span><b style={{ color: '#ffffff' }}>[R]</b> RUN TARGET</span>
          <span><b style={{ color: '#ffffff' }}>[S]</b> STOP TARGET</span>
          <span><b style={{ color: '#ffffff' }}>[0]</b> POP TARGET (5s)</span>
          <span><b style={{ color: '#ffffff' }}>[ESC]</b> EMERGENCY STOP</span>
        </div>

        <div style={{ color: '#9a9486', fontSize: '10px' }}>
          SIMULATION ACTIVE // CLICK TARGET TO ENGAGE
        </div>
      </div>
    </div>
  );
};
