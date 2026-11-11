import { useEffect, useMemo, useRef, useState } from 'react';
import QRCode from 'qrcode';
import type { PendantSession } from './types';
import './pendant.css';

export function PendantIcon() {
  return <svg className="ui-icon" width="17" height="17" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><rect x="5" y="2" width="14" height="20" rx="3"/><path d="M9 6h6M9 12h6M12 9v6M11 18h2"/></svg>;
}
export function PairingQr({ value }: { value: string }) {
  const qr = useMemo(() => {
    const { modules } = QRCode.create(value, { errorCorrectionLevel: 'M' });
    const paths: string[] = [];
    for (let row = 0; row < modules.size; row++) for (let column = 0; column < modules.size; column++) {
      if (modules.get(row, column)) paths.push(`M${column + 4} ${row + 4}h1v1h-1z`);
    }
    return { size: modules.size + 8, path: paths.join('') };
  }, [value]);
  return <svg className="pendant-qr" role="img" aria-label="手机示教器配对二维码" viewBox={`0 0 ${qr.size} ${qr.size}`} shapeRendering="crispEdges"><rect width={qr.size} height={qr.size} fill="#fff"/><path d={qr.path} fill="#111"/></svg>;
}
interface Props {
  session: PendantSession | null;
  busy: boolean;
  error: string;
  onClose(): void;
  onStart(): void;
  onStop(): void;
}
export default function PairingDialog({ session, busy, error, onClose, onStart, onStop }: Props) {
  const dialog = useRef<HTMLDivElement>(null);
  const [addressIndex, setAddressIndex] = useState(0);
  const addresses = session?.urls.filter(entry => !/^127\./.test(new URL(entry.url).hostname) && new URL(entry.url).hostname !== 'localhost') ?? [];
  const address = addresses[Math.min(addressIndex, addresses.length - 1)];
  const [now, setNow] = useState(Date.now());
  useEffect(() => { const timer = setInterval(() => setNow(Date.now()), 1000); return () => clearInterval(timer); }, []);
  useEffect(() => { setAddressIndex(0); }, [session?.sessionId]);
  useEffect(() => {
    const previous = document.activeElement as HTMLElement | null;
    dialog.current?.focus();
    return () => previous?.focus();
  }, []);
  const expired = Boolean(session && !session.connected && now >= session.expiresAtUnixMs);
  return <div className="pendant-backdrop" onMouseDown={event => { if (event.target === event.currentTarget) onClose(); }}>
    <div className="pendant-dialog" ref={dialog} role="dialog" aria-modal="true" aria-labelledby="pendant-dialog-title" tabIndex={-1} onKeyDown={event => {
      if (event.key === 'Escape') { event.stopPropagation(); onClose(); }
      if (event.key !== 'Tab') return;
      const controls = [...event.currentTarget.querySelectorAll<HTMLElement>('button:not(:disabled),select:not(:disabled),a[href]')];
      const first = controls[0], last = controls.at(-1);
      if (event.shiftKey && (document.activeElement === first || document.activeElement === event.currentTarget)) { event.preventDefault(); last?.focus(); }
      else if (!event.shiftKey && (document.activeElement === last || document.activeElement === event.currentTarget)) { event.preventDefault(); first?.focus(); }
    }}>
      <div className="pendant-dialog-heading"><div><span className="pendant-eyebrow">MOBILE PENDANT</span><h2 id="pendant-dialog-title">手机示教器</h2></div><button type="button" className="pendant-close" aria-label="关闭示教器弹窗" onClick={onClose}><svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" aria-hidden="true"><path d="m6 6 12 12M18 6 6 18"/></svg></button></div>
      <p className="pendant-intro">手机与电脑连接同一 Wi-Fi，扫描二维码即可控制当前仿真机器人。</p>
      {error && <p className="pendant-error" role="alert">{error}</p>}
      {busy && !session && <div className="pendant-empty" role="status">正在准备连接…</div>}
      {session && <>
        <div className="pendant-pairing" data-session-connected={session.connected}>
          {address && !expired ? <PairingQr value={address.url}/> : <div className="pendant-empty">{expired ? '二维码已过期，请重新生成。' : '没有可用的局域网地址，请检查网络连接。'}</div>}
          <div className="pendant-pairing-copy"><strong role="status">{session.connected ? '手机已连接' : expired ? '等待重新配对' : '等待手机扫码'}</strong><p>线模式移动 TCP，轴模式调整各关节。拖动摇杆时运动，松手即停。</p><p>一个连接控制一台仿真机器人。关闭此弹窗后仍可操作；“断开示教器”会结束连接。</p></div>
        </div>
        {addresses.length > 1 && <label className="pendant-network">网络地址<select aria-label="示教器网络地址" value={Math.min(addressIndex, addresses.length - 1)} onChange={event => setAddressIndex(Number(event.target.value))}>{addresses.map((entry, index) => <option key={entry.name + index} value={index}>{entry.name} · {new URL(entry.url).host}</option>)}</select></label>}
        {address && <p className="pendant-address">连接地址：<span>{new URL(address.url).origin}</span></p>}
        {!session.connected && <p className="pendant-hint">若无法打开，请确认手机可访问该电脑，并允许系统防火墙中的 Emulation 局域网连接。</p>}
      </>}
      <div className="pendant-dialog-actions">{session ? <><button type="button" disabled={busy} onClick={onStop}>断开示教器</button>{!session.connected && <button type="button" disabled={busy} onClick={onStart}>重新生成二维码</button>}<button type="button" className="pendant-primary" onClick={onClose}>返回场景</button></> : <><button type="button" onClick={onClose}>关闭</button><button type="button" className="pendant-primary" disabled={busy} onClick={onStart}>重新连接</button></>}</div>
    </div>
  </div>;
}
