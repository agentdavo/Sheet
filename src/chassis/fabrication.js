// Welding / fabrication summary for the sheet structure: weld lengths, time, distortion
// risk and material/process warnings. Heuristic guidance - the FE model always assumes
// continuous (seam-welded or bonded) joints.
import { MATERIALS } from '../fea/materials.js';

export const PROCESSES = {
  laser: { label: 'Laser (fibre)', rate: 2.0, heat: 0.2 }, // m/min, relative heat input
  tig: { label: 'TIG', rate: 0.25, heat: 1.0 },
  mig: { label: 'Pulsed MIG', rate: 0.5, heat: 0.7 },
  spot: { label: 'Resistance spot', rate: null, heat: 0.1 },
};
export const JOINING = {
  seam: 'Continuous seam weld',
  stitch: 'Stitch weld',
  spot: 'Spot weld',
};

const MILD = MATERIALS.dc04;

export function fabricationSummary(cfg, mesh) {
  const fab = { joining: 'seam', process: 'tig', stitchLen: 25, stitchPitch: 75, spotPitch: 40, ...(cfg.fabrication || {}) };
  const mat = MATERIALS[cfg.material];
  const f = mesh.fabrication;
  const seam = f.seamLength / 1000; // m of joint line
  const dbl = f.doublerWeld / 1000;
  const frac = fab.joining === 'stitch' ? Math.min(1, fab.stitchLen / fab.stitchPitch) : 1;
  const proc = fab.joining === 'spot' ? PROCESSES.spot : PROCESSES[fab.process] || PROCESSES.tig;
  const weldLen = fab.joining === 'spot' ? 0 : seam * frac + dbl;
  const spots = fab.joining === 'spot' ? Math.round((seam * 1000) / fab.spotPitch) + Math.round((dbl * 1000) / fab.spotPitch) : 0;
  const minutes = fab.joining === 'spot' ? spots / 20 : weldLen / proc.rate;
  const sheetT = Object.entries(cfg.gauges).filter(([g, t]) => g !== 'doublers' && t > 0).map(([, t]) => t);
  const tMin = Math.min(...sheetT);
  const thermal = (mat.alpha / mat.k) / (MILD.alpha / MILD.k);
  const risk = thermal * proc.heat * (1.5 / tMin) ** 2 * (fab.joining === 'stitch' ? 0.6 : 1);
  const level = risk < 2 ? 'low' : risk < 6 ? 'moderate' : 'high';
  const stainless = cfg.material.startsWith('ss');
  const notes = [];
  if (mat.weldNote) notes.push({ level: /not|sensitis|halves/i.test(mat.weldNote) ? 'bad' : 'info', text: `${mat.name}: ${mat.weldNote}` });
  if (stainless && tMin <= 1.5 && fab.joining !== 'spot' && fab.process === 'tig') {
    notes.push({ level: 'warn', text: `TIG on ${tMin} mm stainless: expect significant distortion (expansion ${(mat.alpha * 1e6).toFixed(1)} µm/m/K, conductivity ${mat.k} W/m/K - ${thermal.toFixed(1)}x mild steel's distortion tendency). Fixture with copper chill bars, back-purge, and stitch-weld in a balanced sequence - or laser-weld.` });
  }
  if (fab.joining !== 'seam') {
    notes.push({ level: 'warn', text: `${JOINING[fab.joining]}: the FE model assumes continuous joints. ${fab.joining === 'stitch' ? `${Math.round(frac * 100)}% weld coverage transfers shear only along the stitches` : 'Spot-welded flanges transfer shear through discrete nuggets'} - expect somewhat lower torsional stiffness than predicted (weld-bonding the flanges recovers most of it).` });
  }
  if (!cfg.chassis.beads?.on && tMin <= 1.5) notes.push({ level: 'warn', text: `Flat ${tMin} mm panels without swage beads: low shear-buckling and panel-drumming margins - see the buckling check.` });
  return {
    fab, seam, doublerWeld: dbl, weldLen, spots, minutes, frac, tMin, thermal, risk, level, notes,
    doublerArea: f.doublerArea / 1e6, doublers: f.doublerCount, diaphragms: f.diaphragms, processLabel: proc.label,
  };
}
