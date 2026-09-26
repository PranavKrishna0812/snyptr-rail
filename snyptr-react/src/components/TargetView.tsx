import React, { useRef, useState } from 'react';
import { ElevationState, OptimalROI, PointMm, ShotOutcome, ShotRecord } from '../types';

interface TargetViewProps {
  elevation: ElevationState;
  optimalROI: OptimalROI;
  lastShot: ShotRecord | null;
  recentShots: ShotRecord[];
  isLocked: boolean;
  exposureRemainingSeconds: number;
  onTargetClick: (pointMm: PointMm) => void;
  onPopTarget?: () => void;
}

export const TargetView: React.FC<TargetViewProps> = ({
  elevation,
  optimalROI,
  lastShot,
  recentShots,
  isLocked,
  exposureRemainingSeconds,
  onTargetClick,
  onPopTarget,
}) => {
  const targetRef = useRef<HTMLDivElement>(null);
  const [laserDot, setLaserDot] = useState<{ xMm: number; yMm: number; result: ShotOutcome } | null>(null);

  const handleClick = (e: React.MouseEvent<HTMLDivElement>) => {
    if (isLocked || elevation === 'DOWN') {
      return;
    }

    const rect = targetRef.current?.getBoundingClientRect();
    if (!rect) return;

    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;

    const scale = 150 / rect.width;
    const xMm = Math.max(0, Math.min(150, clickX * scale));
    const yMm = Math.max(0, Math.min(150, clickY * scale));

    const isHit =
      xMm >= optimalROI.x &&
      xMm <= optimalROI.x + optimalROI.width &&
      yMm >= optimalROI.y &&
      yMm <= optimalROI.y + optimalROI.height;

    setLaserDot({ xMm, yMm, result: isHit ? 'HIT' : 'MISS' });
    onTargetClick({ x: Number(xMm.toFixed(1)), y: Number(yMm.toFixed(1)) });
  };

  const toPct = (mm: number) => (mm / 150) * 100;
  const exposurePct = (exposureRemainingSeconds / 5.0) * 100;

  return (
    <div className="ref-panel" style={{
      display: 'flex',
      flexDirection: 'column',
      height: '100%',
      minHeight: '520px',
      padding: '16px',
    }}>
      {/* Header Bar */}
      <div style={{
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'space-between',
        paddingBottom: '10px',
        borderBottom: '1px solid var(--mil-border-light)',
      }}>
        <div style={{
          fontFamily: 'var(--font-mono)',
          fontSize: '12px',
          fontWeight: 800,
          letterSpacing: '0.1em',
          color: 'var(--text-dark)',
          textTransform: 'uppercase',
        }}>
          TARGET // 150 mm × 150 mm
        </div>

        <span style={{
          fontFamily: 'var(--font-mono)',
          fontSize: '11px',
          fontWeight: 800,
          padding: '2px 8px',
          backgroundColor: elevation === 'UP' ? 'var(--mil-hit)' : 'var(--mil-dark)',
          color: '#ffffff',
        }}>
          {elevation === 'UP' ? `EXPOSED [${exposureRemainingSeconds.toFixed(1)}s]` : 'CONCEALED (DOWN)'}
        </span>
      </div>

      {/* Target Canvas Stage */}
      <div style={{
        flex: 1,
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'space-between',
        position: 'relative',
        padding: '16px 0 0 0',
      }}>
        {/* 5-SECOND EXPOSURE PROGRESS BAR */}
        <div style={{ width: '100%', maxWidth: '380px' }}>
          <div style={{
            display: 'flex',
            justifyContent: 'space-between',
            fontFamily: 'var(--font-mono)',
            fontSize: '10px',
            fontWeight: 700,
            color: 'var(--text-muted)',
            marginBottom: '4px',
          }}>
            <span>5.0s EXPOSURE TIMER</span>
            <span style={{ color: 'var(--text-dark)' }}>
              {elevation === 'UP' ? `${exposureRemainingSeconds.toFixed(1)}s REMAINING` : 'READY'}
            </span>
          </div>

          <div style={{
            width: '100%',
            height: '6px',
            backgroundColor: 'var(--mil-canvas-tint)',
            border: '1px solid var(--mil-border-light)',
            overflow: 'hidden',
          }}>
            <div style={{
              width: `${elevation === 'UP' ? exposurePct : 0}%`,
              height: '100%',
              backgroundColor: 'var(--mil-dark)',
              transition: 'width 0.1s linear',
            }} />
          </div>
        </div>

        {/* Physical Target Frame */}
        <div style={{
          position: 'relative',
          padding: '10px',
          backgroundColor: '#2b302a', // dark slate framing
          border: '1px solid #1a1e19',
          boxShadow: '0 4px 16px rgba(0,0,0,0.15)',
          perspective: '800px',
        }}>
          {/* Target Face */}
          <div
            ref={targetRef}
            onClick={handleClick}
            style={{
              width: '350px',
              height: '350px',
              backgroundColor: '#ffffff',
              position: 'relative',
              cursor: isLocked || elevation === 'DOWN' ? 'not-allowed' : 'crosshair',
              border: '1px solid #c8c2b5',
              overflow: 'hidden',
              transition: 'transform 0.28s cubic-bezier(0.2, 0, 0, 1), opacity 0.28s ease',
              transform: elevation === 'DOWN' ? 'rotateX(60deg) scale(0.92) translateY(28px)' : 'none',
              opacity: elevation === 'DOWN' ? 0.35 : 1.0,
            }}
          >
            {/* SVG Target Markings */}
            <svg
              viewBox="0 0 150 150"
              style={{ width: '100%', height: '100%', position: 'absolute', top: 0, left: 0, pointerEvents: 'none' }}
            >
              {/* Outer boundary & crosshairs */}
              <rect x="2" y="2" width="146" height="146" fill="none" stroke="#242923" strokeWidth="1.2" />
              <line x1="75" y1="0" x2="75" y2="150" stroke="#7a8277" strokeWidth="0.5" strokeDasharray="2 2" />
              <line x1="0" y1="75" x2="150" y2="75" stroke="#7a8277" strokeWidth="0.5" strokeDasharray="2 2" />

              {/* Concentric Scoring Rings */}
              <circle cx="75" cy="75" r="66" fill="none" stroke="#242923" strokeWidth="1.0" />
              <circle cx="75" cy="75" r="50" fill="none" stroke="#242923" strokeWidth="1.2" />
              <circle cx="75" cy="75" r="35" fill="none" stroke="#1a1e19" strokeWidth="1.4" />
              <circle cx="75" cy="75" r="20" fill="none" stroke="#1a1e19" strokeWidth="1.6" strokeDasharray="3 1.5" />

              {/* Zone Score Labels */}
              <text x="77" y="13" fill="#242923" fontSize="4.5" fontFamily="monospace" fontWeight="800">ZONE 1</text>
              <text x="77" y="28" fill="#242923" fontSize="4.5" fontFamily="monospace" fontWeight="800">ZONE 2</text>
              <text x="77" y="43" fill="#1a1e19" fontSize="4.5" fontFamily="monospace" fontWeight="800">ZONE 3</text>
              <text x="77" y="58" fill="#1a1e19" fontSize="4.5" fontFamily="monospace" fontWeight="900">ZONE 4</text>

              {/* Bullseye Center */}
              <circle cx="75" cy="75" r="3" fill="#1a1e19" />
              <line x1="70" y1="75" x2="80" y2="75" stroke="#ffffff" strokeWidth="0.75" />
              <line x1="75" y1="70" x2="75" y2="80" stroke="#ffffff" strokeWidth="0.75" />

              {/* Optimal ROI Indicator */}
              <rect
                x={optimalROI.x}
                y={optimalROI.y}
                width={optimalROI.width}
                height={optimalROI.height}
                fill="rgba(44, 89, 52, 0.08)"
                stroke="#2c5934"
                strokeWidth="1.8"
                strokeDasharray="4 2"
              />
              <text
                x={optimalROI.x + 3}
                y={optimalROI.y + 8}
                fill="#2c5934"
                fontSize="4.2"
                fontFamily="monospace"
                fontWeight="900"
              >
                OPTIMAL AREA
              </text>
            </svg>

            {/* Previous Shots Markers */}
            {recentShots.map((shot) => (
              <div
                key={shot.id}
                style={{
                  position: 'absolute',
                  left: `${toPct(shot.xMm)}%`,
                  top: `${toPct(shot.yMm)}%`,
                  width: '8px',
                  height: '8px',
                  transform: 'translate(-50%, -50%)',
                  borderRadius: '50%',
                  backgroundColor: shot.result === 'HIT' ? 'var(--mil-hit)' : 'var(--mil-miss)',
                  border: '1.5px solid #ffffff',
                  pointerEvents: 'none',
                }}
                title={`Shot #${shot.shotNumber}: ${shot.result}`}
              />
            ))}

            {/* Laser Dot Impact */}
            {laserDot && (
              <div
                style={{
                  position: 'absolute',
                  left: `${toPct(laserDot.xMm)}%`,
                  top: `${toPct(laserDot.yMm)}%`,
                  width: '11px',
                  height: '11px',
                  transform: 'translate(-50%, -50%)',
                  borderRadius: '50%',
                  backgroundColor: laserDot.result === 'HIT' ? 'var(--mil-hit)' : 'var(--mil-miss)',
                  boxShadow: '0 0 10px rgba(0,0,0,0.5)',
                  border: '2px solid #ffffff',
                  pointerEvents: 'none',
                  zIndex: 25,
                }}
              />
            )}
          </div>

          {/* OVERLAY WHEN TARGET IS DOWN */}
          {elevation === 'DOWN' && (
            <div style={{
              position: 'absolute',
              top: '50%',
              left: '50%',
              transform: 'translate(-50%, -50%)',
              padding: '16px 20px',
              backgroundColor: '#ffffff',
              border: '2px solid #242923',
              boxShadow: '0 6px 20px rgba(0,0,0,0.2)',
              textAlign: 'center',
              zIndex: 30,
              width: '85%',
            }}>
              <div style={{ fontFamily: 'var(--font-mono)', fontSize: '12px', fontWeight: 900, color: 'var(--text-dark)' }}>
                TARGET CONCEALED [DOWN]
              </div>
              <div style={{ fontSize: '11px', color: 'var(--text-muted)', marginTop: '4px' }}>
                PRESS [0] OR CLICK TO EXPOSE FOR 5 SECONDS
              </div>
              {onPopTarget && (
                <button
                  onClick={onPopTarget}
                  className="ref-btn"
                  style={{ marginTop: '10px' }}
                >
                  POP TARGET [0]
                </button>
              )}
            </div>
          )}
        </div>

        {/* LAST IMPACT VERDICT */}
        <div style={{
          width: '100%',
          maxWidth: '380px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          backgroundColor: '#ffffff',
          border: '1px solid var(--mil-border-light)',
          padding: '10px 14px',
        }}>
          <div>
            <div style={{ fontSize: '10px', color: 'var(--text-muted)', fontFamily: 'var(--font-mono)', fontWeight: 700 }}>
              LAST IMPACT
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '8px', marginTop: '3px' }}>
              {lastShot ? (
                <>
                  <span
                    style={{
                      fontFamily: 'var(--font-mono)',
                      fontSize: '12px',
                      fontWeight: 900,
                      padding: '2px 8px',
                      backgroundColor: lastShot.result === 'HIT' ? 'var(--mil-hit)' : 'var(--mil-miss)',
                      color: '#ffffff',
                    }}
                  >
                    {lastShot.result}
                  </span>
                  <span style={{ fontFamily: 'var(--font-mono)', fontSize: '11px', fontWeight: 700, color: 'var(--text-dark)' }}>
                    {lastShot.distanceFromOptimalMm} mm error ({lastShot.directionFromOptimal})
                  </span>
                </>
              ) : (
                <span style={{ fontFamily: 'var(--font-mono)', fontSize: '11px', color: 'var(--text-muted)' }}>
                  AWAITING LASER IMPACT
                </span>
              )}
            </div>
          </div>

          {lastShot && (
            <div style={{ textAlign: 'right', fontFamily: 'var(--font-mono)', fontSize: '11px', fontWeight: 700, color: 'var(--text-muted)' }}>
              <div>{lastShot.positionMeters.toFixed(3)} m</div>
              <div>X:{lastShot.xMm} Y:{lastShot.yMm}</div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
