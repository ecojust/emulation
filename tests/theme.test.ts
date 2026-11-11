import { readFileSync } from 'node:fs';
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { getTheme, initializeTheme, parseTheme, setTheme, THEME_CHANGE_EVENT, THEME_STORAGE_KEY } from '../src/theme/theme';

const css = readFileSync(new URL('../src/theme/theme.css', import.meta.url), 'utf8');
function palette(theme: 'light' | 'dark') {
  const block = css.match(new RegExp(`\\[data-theme='${theme}'\\]\\s*\\{([^}]+)\\}`))![1];
  return Object.fromEntries([...block.matchAll(/(--[a-z-]+):\s*(#[a-f0-9]{6})\s*;/g)].map(match => [match[1], match[2]]));
}
function contrast(a: string, b: string) {
  const luminance = (hex: string) => {
    const [r, g, blue] = [1, 3, 5].map(index => parseInt(hex.slice(index, index + 2), 16) / 255)
      .map(value => value <= .04045 ? value / 12.92 : ((value + .055) / 1.055) ** 2.4);
    return .2126 * r + .7152 * g + .0722 * blue;
  };
  const values = [luminance(a), luminance(b)].sort((x, y) => y - x);
  return (values[0] + .05) / (values[1] + .05);
}

describe('persistent application theme', () => {
  let saved: Map<string, string>;
  beforeEach(() => {
    saved = new Map();
    vi.stubGlobal('document', { documentElement: { dataset: {}, style: {} } });
    vi.stubGlobal('window', new EventTarget());
    vi.stubGlobal('localStorage', { getItem: (key: string) => saved.get(key) ?? null, setItem: (key: string, value: string) => saved.set(key, value) });
  });
  afterEach(() => vi.unstubAllGlobals());
  it('starts light without a preference and applies a saved dark preference before mounting', () => {
    expect(initializeTheme()).toBe('light');
    saved.set(THEME_STORAGE_KEY, 'dark');
    expect(initializeTheme()).toBe('dark');
    expect(getTheme()).toBe('dark');
    expect(document.documentElement.style.colorScheme).toBe('dark');
  });
  it('persists explicit switching and notifies the mounted scene without recreating it', () => {
    const onChange = vi.fn(); window.addEventListener(THEME_CHANGE_EVENT, onChange);
    setTheme('dark');
    expect(saved.get(THEME_STORAGE_KEY)).toBe('dark');
    expect(onChange).toHaveBeenCalledOnce();
    setTheme('light');
    expect(getTheme()).toBe('light');
    expect(saved.get(THEME_STORAGE_KEY)).toBe('light');
  });
  it('rejects unknown preferences and remains usable when browser storage is blocked', () => {
    expect(parseTheme('auto')).toBeNull();
    expect(parseTheme({ theme: 'dark' })).toBeNull();
    saved.set(THEME_STORAGE_KEY, 'unsupported');
    expect(initializeTheme()).toBe('light');
    vi.stubGlobal('localStorage', { getItem: () => { throw new Error('blocked'); }, setItem: () => { throw new Error('blocked'); } });
    expect(() => initializeTheme()).not.toThrow();
    expect(() => setTheme('dark')).not.toThrow();
    expect(getTheme()).toBe('dark');
  });
});

for (const theme of ['light', 'dark'] as const) {
  it(`${theme} palette keeps normal text and status text above 4.5:1 contrast`, () => {
    const colors = palette(theme);
    for (const background of ['--surface', '--surface-subtle', '--surface-raised', '--page']) {
      for (const foreground of ['--text', '--text-secondary', '--muted', '--accent', '--danger', '--warning', '--success']) {
        expect(contrast(colors[foreground], colors[background]), `${theme} ${foreground} on ${background}`).toBeGreaterThanOrEqual(4.5);
      }
    }
    for (const role of ['accent', 'danger', 'warning', 'success']) {
      expect(contrast(colors[`--${role}`], colors[`--${role}-soft`]), `${theme} ${role} message`).toBeGreaterThanOrEqual(4.5);
      expect(contrast(colors['--on-accent'], colors[`--${role}-fill`]), `${theme} ${role} button`).toBeGreaterThanOrEqual(4.5);
    }
    for (const token of ['keyword', 'type', 'string', 'number', 'comment']) {
      expect(contrast(colors[`--syntax-${token}`], colors['--surface']), `${theme} syntax ${token}`).toBeGreaterThanOrEqual(4.5);
    }
  });
}
