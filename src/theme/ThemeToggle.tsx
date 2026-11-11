import { setTheme, useTheme } from './theme';

/** Shared by the project home and workspace; the preference belongs to the app. */
export default function ThemeToggle() {
  const theme = useTheme(), dark = theme === 'dark';
  return <button type="button" className="theme-toggle" aria-label="切换主题" aria-pressed={dark}
    title={dark ? '切换为浅色主题' : '切换为深色主题'} onClick={() => setTheme(dark ? 'light' : 'dark')}>
    <svg className="ui-icon" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
      {dark ? <path d="M20.2 15.4A8.5 8.5 0 0 1 8.6 3.8 8.5 8.5 0 1 0 20.2 15.4Z"/> : <><circle cx="12" cy="12" r="4"/><path d="M12 2v2m0 16v2M2 12h2m16 0h2M4.9 4.9l1.4 1.4m11.4 11.4 1.4 1.4M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4"/></>}
    </svg><span>{dark ? '深色' : '浅色'}</span>
  </button>;
}
