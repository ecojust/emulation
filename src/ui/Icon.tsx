import type { ReactNode, SVGProps } from 'react';
import './icons.css';

const shapes = {
  cube: <><path d="m12 3 8 4.5v9L12 21l-8-4.5v-9Z"/><path d="m4 7.5 8 4.5 8-4.5M12 12v9M8 5.3l8 4.5"/></>,
  arrow: <path d="M5 12h14m-6-6 6 6-6 6"/>,
  play: <path d="m8 5 11 7-11 7Z"/>,
  pause: <path d="M8 5v14M16 5v14"/>,
  stop: <rect x="6" y="6" width="12" height="12" rx="1"/>,
  copy: <><rect x="8" y="8" width="12" height="13" rx="2"/><path d="M16 8V5a2 2 0 0 0-2-2H5a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h3"/></>,
  reset: <path d="M3 10a9 9 0 1 1 2 8M3 4v6h6"/>,
  check: <path d="m5 12 4 4L19 6"/>,
  target: <><circle cx="12" cy="12" r="7"/><circle cx="12" cy="12" r="2"/><path d="M12 2v3m0 14v3M2 12h3m14 0h3"/></>,
  code: <path d="m8 6-6 6 6 6m8-12 6 6-6 6M14 3l-4 18"/>,
  info: <><circle cx="12" cy="12" r="9"/><path d="M12 11v6m0-10v.01"/></>,
  plus: <path d="M12 5v14M5 12h14"/>,
  minus: <path d="M5 12h14"/>,
  'chevron-left': <path d="m15 5-7 7 7 7"/>,
  'chevron-right': <path d="m9 5 7 7-7 7"/>,
  'chevron-down': <path d="m5 9 7 7 7-7"/>,
  'chevron-up': <path d="m5 15 7-7 7 7"/>,
  close: <path d="m6 6 12 12M6 18 18 6"/>,
  warning: <><path d="M10.3 3.9a2 2 0 0 1 3.4 0l8 14A2 2 0 0 1 20 21H4a2 2 0 0 1-1.7-3.1Z"/><path d="M12 9v5m0 3v.01"/></>,
  question: <><circle cx="12" cy="12" r="9"/><path d="M9.5 9a2.5 2.5 0 1 1 4 2c-1 .7-1.5 1.2-1.5 2.5m0 3v.01"/></>,
  circle: <circle cx="12" cy="12" r="8"/>,
  dot: <circle cx="12" cy="12" r="5" fill="currentColor" stroke="none"/>,
  path: <><circle cx="5" cy="17" r="2"/><circle cx="19" cy="7" r="2"/><path d="M5 15V9a4 4 0 0 1 8 0v6a4 4 0 0 0 8 0v-3"/></>,
  route: <path d="M2 12h20" strokeDasharray="4 3"/>,
  zone: <rect x="4" y="4" width="16" height="16" rx="2"/>,
  robot: <><path d="M3 21h11M5 18h7v3M8 18v-5l8-6m-7 3L5 6m10-1 4-3 3 4-4 3"/><circle cx="8" cy="11" r="3"/><circle cx="16" cy="6" r="2"/></>,
  file: <><path d="M14 2H5v20h14V7Z"/><path d="M14 2v5h5M8 12h8M8 16h6"/></>,
  expand: <path d="M8 3H3v5m13-5h5v5M3 16v5h5m13-5v5h-5"/>,
  collapse: <path d="M3 8h5V3m8 0v5h5M8 21v-5H3m13 5v-5h5"/>,
  search: <><circle cx="10.5" cy="10.5" r="6.5"/><path d="m16 16 5 5"/></>,
  trash: <><path d="M3 6h18M8 6V3h8v3M5 6l1 15h12l1-15M10 10v7m4-7v7"/></>,
  download: <><path d="M12 3v12m-5-5 5 5 5-5M4 16v5h16v-5"/></>,
} satisfies Record<string, ReactNode>;

export type IconName = keyof typeof shapes;
export type IconProps = Omit<SVGProps<SVGSVGElement>, 'name'> & { name: IconName; size?: number };

/** Shared UI artwork. Accessible labels belong to the enclosing control. */
export default function Icon({ name, size = 18, className = '', ...props }: IconProps) {
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6"
    strokeLinecap="round" strokeLinejoin="round" aria-hidden="true" focusable="false"
    className={`ui-icon${className ? ' ' + className : ''}`} data-icon={name} {...props}>{shapes[name]}</svg>;
}
