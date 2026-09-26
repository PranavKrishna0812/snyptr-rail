import React from 'react';

interface SessionStatsProps {
  totalShots: number;
  maxShots?: number;
  hits: number;
  misses: number;
  accuracy: number;
  onStartNewSession: () => void;
}

export const SessionStats: React.FC<SessionStatsProps> = ({
  totalShots,
  maxShots = 20,
  hits,
  misses,
  accuracy,
}) => {
  return (
    <div style={{
      display: 'grid',
      gridTemplateColumns: '1.2fr 1fr 1fr 1fr',
      gap: '12px',
      width: '100%',
    }}>
      {/* 20-Shot Progression */}
      <div className="ref-panel" style={{
        padding: '12px 18px',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        backgroundColor: '#ffffff',
      }}>
        <div>
          <div style={{ fontSize: '10px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700 }}>
            ENGAGEMENT
          </div>
          <div style={{
            fontFamily: 'var(--font-mono)',
            fontSize: '26px',
            fontWeight: 900,
            color: 'var(--text-dark)',
            lineHeight: 1.0,
            marginTop: '3px',
          }}>
            SHOT {String(totalShots).padStart(2, '0')} <span style={{ color: 'var(--text-muted)', fontSize: '16px', fontWeight: 600 }}>/ {maxShots}</span>
          </div>
        </div>

        <div style={{ textAlign: 'right' }}>
          <div style={{ fontSize: '9px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700 }}>
            REMAINING
          </div>
          <div style={{
            fontFamily: 'var(--font-mono)',
            fontSize: '20px',
            fontWeight: 800,
            color: 'var(--mil-dark)',
            marginTop: '2px',
          }}>
            {Math.max(0, maxShots - totalShots)}
          </div>
        </div>
      </div>

      {/* Hits */}
      <div className="ref-panel" style={{
        padding: '12px 18px',
        backgroundColor: '#ffffff',
        borderLeft: '4px solid var(--mil-hit)',
      }}>
        <div style={{ fontSize: '10px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700 }}>
          HITS
        </div>
        <div style={{
          fontFamily: 'var(--font-mono)',
          fontSize: '26px',
          fontWeight: 900,
          color: 'var(--mil-hit)',
          marginTop: '3px',
          lineHeight: 1.0,
        }}>
          {hits}
        </div>
      </div>

      {/* Misses */}
      <div className="ref-panel" style={{
        padding: '12px 18px',
        backgroundColor: '#ffffff',
        borderLeft: '4px solid var(--mil-miss)',
      }}>
        <div style={{ fontSize: '10px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700 }}>
          MISSES
        </div>
        <div style={{
          fontFamily: 'var(--font-mono)',
          fontSize: '26px',
          fontWeight: 900,
          color: 'var(--mil-miss)',
          marginTop: '3px',
          lineHeight: 1.0,
        }}>
          {misses}
        </div>
      </div>

      {/* Accuracy */}
      <div className="ref-panel" style={{
        padding: '12px 18px',
        backgroundColor: '#ffffff',
        borderLeft: '4px solid var(--mil-dark)',
      }}>
        <div style={{ fontSize: '10px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700 }}>
          ACCURACY
        </div>
        <div style={{
          fontFamily: 'var(--font-mono)',
          fontSize: '26px',
          fontWeight: 900,
          color: 'var(--text-dark)',
          marginTop: '3px',
          lineHeight: 1.0,
        }}>
          {totalShots > 0 ? `${accuracy.toFixed(1)}%` : '0.0%'}
        </div>
      </div>
    </div>
  );
};
