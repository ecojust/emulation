import { useSyncExternalStore } from 'react';
import './theme.css';

export type Theme = 'light' | 'dark';
export const THEME_STORAGE_KEY = 'emulation.theme';
export const THEME_CHANGE_EVENT = 'emulation-theme-change';

export function parseTheme(value: unknown): Theme | null {
  return value === 'light' || value === 'dark' ? value : null;
}

export function getTheme(): Theme {
  return typeof document === 'undefined' ? 'light' : parseTheme(document.documentElement.dataset.theme) ?? 'light';
}

function applyTheme(theme: Theme): void {
  document.documentElement.dataset.theme = theme;
  document.documentElement.style.colorScheme = theme;
  window.dispatchEvent(new Event(THEME_CHANGE_EVENT));
}

/** Called before mounting React so reopening the app never flashes another theme. */
export function initializeTheme(): Theme {
  let saved: Theme | null = null;
  try { saved = parseTheme(localStorage.getItem(THEME_STORAGE_KEY)); } catch { /* Storage can be unavailable in a private WebView. */ }
  const theme = saved ?? 'light';
  if (typeof document !== 'undefined') applyTheme(theme);
  return theme;
}

export function setTheme(theme: Theme): void {
  if (!parseTheme(theme)) return;
  if (typeof document !== 'undefined') applyTheme(theme);
  try { localStorage.setItem(THEME_STORAGE_KEY, theme); } catch { /* Switching still works for this app session. */ }
}

function subscribe(listener: () => void): () => void {
  const storage = (event: StorageEvent) => {
    if (event.key !== THEME_STORAGE_KEY && event.key !== null) return;
    const theme = parseTheme(event.newValue) ?? 'light';
    applyTheme(theme);
  };
  window.addEventListener(THEME_CHANGE_EVENT, listener);
  window.addEventListener('storage', storage);
  return () => {
    window.removeEventListener(THEME_CHANGE_EVENT, listener);
    window.removeEventListener('storage', storage);
  };
}

export function useTheme(): Theme {
  return useSyncExternalStore(subscribe, getTheme, () => 'light');
}
