import React from 'react';
import { TargetState } from '../types';

interface RailPositionControlProps {
  targetState: TargetState;
  onRun: () => void;
  onStop: () => void;
  onPop: () => void;
  onEmergencyStop: () => void;
}

export const RailPositionControl: React.FC<RailPositionControlProps> = ({
  targetState,
  onRun,
  onStop,
  onPop,
  onEmergencyStop,
}) => {
  const { positionMeters, direction, elevation, systemStatus, exposureRemainingSeconds } = targetState;
  
  const positionPct = Math.min(100, Math.max(0, (positionMeters / 2.0) * 100));
  const isMoving = direction !== 'STOPPED';

  return (
    <div className="ref-panel" style={{ display: 'flex', flexDirection: 'column', height: '100%', padding: '16px' }}>
      {/* 3 HERO TILES (MATCHING REFERENCE IMAGE 3-TILE LAYOUT) */}
      <div>
        <div style={{ display: 'grid', gridTemplateColumns: 'repeat(3, 1fr)', gap: '12px' }}>
          {/* TILE 1: [R] RUN */}
          <button
            onClick={onRun}
            className={`ref-cmd-tile ${isMoving ? 'active-run' : ''}`}
            disabled={systemStatus === 'EMERGENCY_STOP'}
            title="Global shortcut: R"
          >
            <div className="ref-tile-key">R</div>
            <div className="ref-tile-title">
              {isMoving ? 'RUNNING' : 'RUN TARGET'}
            </div>
            <div className="ref-tile-sub">
              {isMoving ? (direction === 'FORWARD' ? 'AWAY ▲' : 'CLOSER ▼') : 'START DRIVE'}
            </div>
          </button>

          {/* TILE 2: [S] STOP */}
          <button
            onClick={onStop}
            className={`ref-cmd-tile ${!isMoving && systemStatus !== 'EMERGENCY_STOP' ? 'active-stop' : ''}`}
            disabled={systemStatus === 'EMERGENCY_STOP'}
            title="Global shortcut: S"
          >
            <div className="ref-tile-key">S</div>
            <div className="ref-tile-title">STOP TARGET</div>
            <div className="ref-tile-sub">HALT MOTION</div>
          </button>

          {/* TILE 3: [0] POP (5s AUTO-DROP) */}
          <button
            onClick={onPop}
            className={`ref-cmd-tile ${elevation === 'UP' ? 'active-pop' : ''}`}
            disabled={systemStatus === 'EMERGENCY_STOP'}
            title="Global shortcut: 0"
          >
            <div className="ref-tile-key">0</div>
            <div className="ref-tile-title">
              {elevation === 'UP' ? `TARGET UP (${exposureRemainingSeconds}s)` : 'POP TARGET (5s)'}
            </div>
            <div className="ref-tile-sub">
              {elevation === 'UP' ? 'EXPOSURE ACTIVE' : 'SERVO FOLDED'}
            </div>
          </button>
        </div>

        {/* Full-Width ESTOP Button */}
        <button
          onClick={onEmergencyStop}
          className="ref-btn"
          style={{
            width: '100%',
            marginTop: '10px',
            backgroundColor: systemStatus === 'EMERGENCY_STOP' ? 'var(--mil-miss)' : '#2e342d',
            padding: '10px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            gap: '10px',
          }}
          title="Global shortcut: ESC"
        >
          <span style={{ fontWeight: 900, fontSize: '13px', color: '#ff7777' }}>[ESC]</span>
          <span>EMERGENCY STOP // HALT CARRIAGE</span>
        </button>
      </div>

      {/* CONTINUOUS DISTANCE & RAIL TRACK */}
      <div style={{
        marginTop: '16px',
        backgroundColor: '#ffffff',
        border: '1px solid var(--mil-border-light)',
        padding: '16px 20px',
        display: 'flex',
        flexDirection: 'column',
        gap: '14px',
      }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-end' }}>
          <div>
            <div style={{ fontSize: '11px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700, letterSpacing: '0.08em' }}>
              TARGET DISTANCE FROM SHOOTER
            </div>
            <div style={{
              fontFamily: 'var(--font-mono)',
              fontSize: '38px',
              fontWeight: 900,
              color: 'var(--text-dark)',
              letterSpacing: '0.02em',
              lineHeight: 1.0,
              marginTop: '4px',
            }}>
              {positionMeters.toFixed(3)} <span style={{ fontSize: '18px', fontWeight: 700, color: 'var(--text-muted)' }}>m</span>
            </div>
          </div>

          <div style={{ textAlign: 'right' }}>
            <div style={{ fontSize: '10px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700 }}>
              CARRIAGE STATUS
            </div>
            <div style={{
              fontFamily: 'var(--font-mono)',
              fontSize: '12px',
              fontWeight: 800,
              color: 'var(--mil-dark)',
              marginTop: '4px',
              backgroundColor: 'var(--mil-canvas-tint)',
              padding: '3px 8px',
              border: '1px solid var(--mil-border-light)',
            }}>
              {isMoving ? (direction === 'FORWARD' ? 'MOVING AWAY ▲' : 'RETURNING ▼') : 'STOPPED'} // {elevation}
            </div>
          </div>
        </div>

        {/* Precision Rail Track */}
        <div>
          <div style={{
            display: 'flex',
            justifyContent: 'space-between',
            fontSize: '11px',
            fontFamily: 'var(--font-mono)',
            fontWeight: 800,
            color: 'var(--text-muted)',
            marginBottom: '6px',
          }}>
            <span>[SHOOTER] 0.0 m</span>
            <span>2.0 m [AWAY]</span>
          </div>

          <div style={{
            position: 'relative',
            height: '40px',
            backgroundColor: 'var(--mil-card-bg)',
            border: '1px solid var(--mil-border-light)',
            display: 'flex',
            alignItems: 'center',
          }}>
            {/* Metric Milestones */}
            <div style={{ position: 'absolute', left: '25%', height: '100%', width: '1px', backgroundColor: 'var(--mil-border-light)' }}>
              <span style={{ position: 'absolute', top: '2px', left: '4px', fontSize: '9px', fontFamily: 'var(--font-mono)', color: 'var(--text-muted)' }}>0.5m</span>
            </div>
            <div style={{ position: 'absolute', left: '50%', height: '100%', width: '1px', backgroundColor: 'var(--mil-border-light)' }}>
              <span style={{ position: 'absolute', top: '2px', left: '4px', fontSize: '9px', fontFamily: 'var(--font-mono)', color: 'var(--text-muted)' }}>1.0m</span>
            </div>
            <div style={{ position: 'absolute', left: '75%', height: '100%', width: '1px', backgroundColor: 'var(--mil-border-light)' }}>
              <span style={{ position: 'absolute', top: '2px', left: '4px', fontSize: '9px', fontFamily: 'var(--font-mono)', color: 'var(--text-muted)' }}>1.5m</span>
            </div>

            {/* Guide Rail */}
            <div style={{
              position: 'absolute',
              top: '18px',
              left: '8px',
              right: '8px',
              height: '3px',
              backgroundColor: '#353c34',
            }} />

            {/* Target Carriage */}
            <div style={{
              position: 'absolute',
              left: `${positionPct}%`,
              transform: 'translateX(-50%)',
              display: 'flex',
              flexDirection: 'column',
              alignItems: 'center',
              zIndex: 10,
              transition: 'left 0.05s linear',
            }}>
              <div style={{
                width: '24px',
                height: '24px',
                backgroundColor: elevation === 'UP' ? 'var(--mil-hit)' : '#9e7a3d',
                border: '2px solid #ffffff',
                boxShadow: '0 2px 6px rgba(0,0,0,0.3)',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
              }}>
                <div style={{ width: '6px', height: '6px', backgroundColor: '#ffffff' }} />
              </div>
              <div style={{
                fontSize: '10px',
                fontFamily: 'var(--font-mono)',
                fontWeight: 900,
                color: '#ffffff',
                backgroundColor: 'var(--mil-dark)',
                padding: '1px 5px',
                marginTop: '2px',
                whiteSpace: 'nowrap',
              }}>
                {positionMeters.toFixed(2)}m
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};
