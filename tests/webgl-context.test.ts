import { afterEach, describe, expect, it, vi } from 'vitest';
import { WebGLRenderer } from 'three';
import { createValidatedRenderer, hasUsableWebGL2Capabilities } from '../src/scene/webgl-context';

vi.mock('three', () => ({ WebGLRenderer: vi.fn(function (this: { settings: unknown }, settings: unknown) { this.settings = settings; }) }));
afterEach(() => { vi.unstubAllGlobals(); vi.clearAllMocks(); });

function context(options: { nullPrecision?: boolean; lost?: boolean; attributes?: number; mediumOnly?: boolean } = {}) {
  const loseContext = vi.fn();
  const gl = {
    VERTEX_SHADER: 1, FRAGMENT_SHADER: 2, HIGH_FLOAT: 3, MEDIUM_FLOAT: 4,
    MAX_VERTEX_ATTRIBS: 5, MAX_TEXTURE_SIZE: 6,
    isContextLost: () => options.lost ?? false,
    getContextAttributes: () => ({ alpha: true }),
    getShaderPrecisionFormat: (_shader: number, precision: number) => options.nullPrecision ? null : ({ precision: options.mediumOnly && precision === 3 ? 0 : 23 }),
    getParameter: (key: number) => key === 5 ? options.attributes ?? 16 : 8192,
    getExtension: () => ({ loseContext }),
  } as unknown as WebGL2RenderingContext;
  return { gl, loseContext };
}

function canvasFor(gl: WebGL2RenderingContext | null) {
  return { getContext: vi.fn(() => gl), remove: vi.fn() };
}

describe('WebGL2 renderer initialization', () => {
  it('rejects the observed non-null context with null shader precision before constructing Three, then uses a fresh fallback canvas', () => {
    const broken = context({ nullPrecision: true }), working = context();
    const first = canvasFor(broken.gl), second = canvasFor(working.gl);
    const createElement = vi.fn().mockReturnValueOnce(first).mockReturnValueOnce(second);
    vi.stubGlobal('document', { createElement });
    const result = createValidatedRenderer();
    expect(result.compatibility).toBe(true);
    expect(createElement).toHaveBeenCalledTimes(2);
    expect(first.getContext).toHaveBeenCalledWith('webgl2', { alpha: true, antialias: true, powerPreference: 'high-performance' });
    expect(second.getContext).toHaveBeenCalledWith('webgl2', { alpha: true, antialias: false, powerPreference: 'default' });
    expect(broken.loseContext).toHaveBeenCalledOnce();
    expect(first.remove).toHaveBeenCalledOnce();
    expect(WebGLRenderer).toHaveBeenCalledTimes(1);
    expect(WebGLRenderer).toHaveBeenCalledWith(expect.objectContaining({ canvas: second, context: working.gl, antialias: false, precision: 'highp' }));
    expect(working.loseContext).not.toHaveBeenCalled();
  });

  it('keeps the preferred context when its real capabilities are usable', () => {
    const working = context(), canvas = canvasFor(working.gl), createElement = vi.fn(() => canvas);
    vi.stubGlobal('document', { createElement });
    expect(createValidatedRenderer().compatibility).toBe(false);
    expect(createElement).toHaveBeenCalledOnce();
    expect(WebGLRenderer).toHaveBeenCalledWith(expect.objectContaining({ context: working.gl, antialias: true, powerPreference: 'high-performance' }));
    expect(working.loseContext).not.toHaveBeenCalled();
  });

  it('reports a real failure after exactly two unusable contexts and never creates a fake renderer', () => {
    const firstContext = context({ nullPrecision: true }), secondContext = context({ lost: true });
    const createElement = vi.fn().mockReturnValueOnce(canvasFor(firstContext.gl)).mockReturnValueOnce(canvasFor(secondContext.gl));
    vi.stubGlobal('document', { createElement });
    expect(() => createValidatedRenderer()).toThrow('两种图形配置均未提供有效的 WebGL 2');
    expect(createElement).toHaveBeenCalledTimes(2);
    expect(WebGLRenderer).not.toHaveBeenCalled();
    expect(firstContext.loseContext).toHaveBeenCalledOnce();
    expect(secondContext.loseContext).toHaveBeenCalledOnce();
  });

  it('rejects context loss and invalid vertex limits even when a context object exists', () => {
    expect(hasUsableWebGL2Capabilities(context({ lost: true }).gl)).toBe(false);
    expect(hasUsableWebGL2Capabilities(context({ attributes: 0 }).gl)).toBe(false);
    expect(hasUsableWebGL2Capabilities(context({ attributes: NaN }).gl)).toBe(false);
    expect(hasUsableWebGL2Capabilities(context({ attributes: Infinity }).gl)).toBe(false);
  });

  it('uses the supported medium precision without inventing high-precision capability', () => {
    const working = context({ mediumOnly: true });
    vi.stubGlobal('document', { createElement: vi.fn(() => canvasFor(working.gl)) });
    createValidatedRenderer();
    expect(WebGLRenderer).toHaveBeenCalledWith(expect.objectContaining({ precision: 'mediump', context: working.gl }));
  });
});
