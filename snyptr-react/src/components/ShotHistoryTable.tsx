import React from 'react';
import { ShotRecord } from '../types';

interface ShotHistoryTableProps {
  shots: ShotRecord[];
}

export const ShotHistoryTable: React.FC<ShotHistoryTableProps> = ({ shots }) => {
  return (
    <div className="ref-panel" style={{
      display: 'flex',
      flexDirection: 'column',
      height: '100%',
      padding: '14px 16px',
    }}>
      <div style={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        paddingBottom: '8px',
        borderBottom: '1px solid var(--mil-border-light)',
        marginBottom: '8px',
      }}>
        <div style={{
          fontFamily: 'var(--font-mono)',
          fontSize: '11px',
          fontWeight: 800,
          letterSpacing: '0.08em',
          color: 'var(--text-dark)',
          textTransform: 'uppercase',
        }}>
          RECENT IMPACT LOG
        </div>
        <span style={{ fontSize: '10px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)' }}>
          {shots.length} OF 20 RECORDED
        </span>
      </div>

      <div style={{ flex: 1, overflowY: 'auto', display: 'flex', flexDirection: 'column', gap: '6px' }}>
        {shots.length === 0 ? (
          <div style={{
            height: '100%',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            color: 'var(--text-muted)',
            fontFamily: 'var(--font-mono)',
            fontSize: '11px',
            fontStyle: 'italic',
            textAlign: 'center',
            padding: '16px',
          }}>
            NO SHOTS LOGGED // CLICK TARGET TO RECORD IMPACT
          </div>
        ) : (
          [...shots].reverse().slice(0, 5).map((shot) => (
            <div
              key={shot.id}
              style={{
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'space-between',
                padding: '6px 12px',
                backgroundColor: '#ffffff',
                border: '1px solid var(--mil-border-light)',
                fontFamily: 'var(--font-mono)',
                fontSize: '11px',
              }}
            >
              <div style={{ display: 'flex', alignItems: 'center', gap: '12px' }}>
                <span style={{ fontWeight: 800, color: 'var(--text-dark)' }}>
                  #{String(shot.shotNumber).padStart(2, '0')}
                </span>

                <span
                  style={{
                    fontWeight: 900,
                    padding: '2px 8px',
                    backgroundColor: shot.result === 'HIT' ? 'var(--mil-hit)' : 'var(--mil-miss)',
                    color: '#ffffff',
                    fontSize: '10px',
                  }}
                >
                  {shot.result}
                </span>

                <span style={{ fontWeight: 700, color: 'var(--text-dark)' }}>
                  {shot.positionMeters.toFixed(3)} m
                </span>
              </div>

              <div style={{ display: 'flex', alignItems: 'center', gap: '16px' }}>
                <span style={{ color: 'var(--text-muted)' }}>
                  {shot.distanceFromOptimalMm.toFixed(1)} mm ({shot.directionFromOptimal})
                </span>
                <span style={{ color: 'var(--text-muted)', fontSize: '10px' }}>
                  {shot.timestamp}
                </span>
              </div>
            </div>
          ))
        )}
      </div>
    </div>
  );
};
