import React from 'react';
import { TargetState } from '../types';

interface SystemStatusPanelProps {
  targetState: TargetState;
}

export const SystemStatusPanel: React.FC<SystemStatusPanelProps> = ({ targetState }) => {
  return (
    <div className="mil-card" style={{ display: 'flex', flexDirection: 'column' }}>
      <div className="mil-card-header" style={{ padding: '6px 12px' }}>
        <span>SUBSYSTEM TELEMETRY</span>
        <span className="badge badge-outline" style={{ fontSize: '9px', padding: '1px 6px' }}>
          HARDWARE SIMULATION BRIDGE
        </span>
      </div>

      <div style={{
        padding: '8px 12px',
        display: 'grid',
        gridTemplateColumns: 'repeat(4, 1fr)',
        gap: '8px',
        fontSize: '11px',
        fontFamily: 'var(--font-mono)',
        backgroundColor: '#ffffff',
      }}>
        <div style={{ padding: '6px 8px', backgroundColor: 'var(--mil-green-ghost)', border: '1px solid var(--border-medium)' }}>
          <div style={{ color: 'var(--text-muted)', fontSize: '9px', fontWeight: 700 }}>SERVO (MG90S)</div>
          <div style={{ color: 'var(--mil-green-deep)', fontWeight: 800, marginTop: '2px' }}>
            {targetState.elevation === 'UP' ? `UP [${targetState.exposureRemainingSeconds}s]` : 'DOWN (FOLDED)'}
          </div>
        </div>

        <div style={{ padding: '6px 8px', backgroundColor: 'var(--mil-green-ghost)', border: '1px solid var(--border-medium)' }}>
          <div style={{ color: 'var(--text-muted)', fontSize: '9px', fontWeight: 700 }}>RANGE (VL53L0X)</div>
          <div style={{ color: 'var(--mil-green-deep)', fontWeight: 800, marginTop: '2px' }}>
            {targetState.positionMeters.toFixed(3)} m
          </div>
        </div>

        <div style={{ padding: '6px 8px', backgroundColor: 'var(--mil-green-ghost)', border: '1px solid var(--border-medium)' }}>
          <div style={{ color: 'var(--text-muted)', fontSize: '9px', fontWeight: 700 }}>CAMERA (OV5647)</div>
          <div style={{ color: 'var(--mil-green-deep)', fontWeight: 800, marginTop: '2px' }}>
            SIM ONLINE
          </div>
        </div>

        <div style={{ padding: '6px 8px', backgroundColor: 'var(--mil-green-ghost)', border: '1px solid var(--border-medium)' }}>
          <div style={{ color: 'var(--text-muted)', fontSize: '9px', fontWeight: 700 }}>WIRELESS (ESP32-S3)</div>
          <div style={{ color: 'var(--mil-green-deep)', fontWeight: 800, marginTop: '2px' }}>
            LINK ACTIVE
          </div>
        </div>
      </div>
    </div>
  );
};
