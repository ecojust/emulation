import { WebGLRenderer } from 'three';

/** A non-null WebGL2 handle can still be unusable after a GPU/driver initialization failure. */
export function hasUsableWebGL2Capabilities(context: WebGL2RenderingContext): boolean {
  try {
    if (context.isContextLost() || !context.getContextAttributes()) return false;
    const vertexHigh = context.getShaderPrecisionFormat(context.VERTEX_SHADER, context.HIGH_FLOAT);
    const fragmentHigh = context.getShaderPrecisionFormat(context.FRAGMENT_SHADER, context.HIGH_FLOAT);
    const vertexMedium = context.getShaderPrecisionFormat(context.VERTEX_SHADER, context.MEDIUM_FLOAT);
    const fragmentMedium = context.getShaderPrecisionFormat(context.FRAGMENT_SHADER, context.MEDIUM_FLOAT);
    if (!vertexHigh || !fragmentHigh || !vertexMedium || !fragmentMedium) return false;
    const workingPrecision = (vertexHigh.precision > 0 && fragmentHigh.precision > 0)
      || (vertexMedium.precision > 0 && fragmentMedium.precision > 0);
    const attributes = context.getParameter(context.MAX_VERTEX_ATTRIBS);
    const textureSize = context.getParameter(context.MAX_TEXTURE_SIZE);
    return workingPrecision && typeof attributes === 'number' && Number.isFinite(attributes) && attributes > 0
      && typeof textureSize === 'number' && Number.isFinite(textureSize) && textureSize > 0;
  } catch { return false; }
}

function release(context: WebGL2RenderingContext | null, canvas: HTMLCanvasElement): void {
  try { context?.getExtension('WEBGL_lose_context')?.loseContext(); } catch { /* Failed contexts may reject extension queries. */ }
  canvas.remove();
}

/** Exactly two genuine WebGL2 attempts. No UA/platform exceptions or suppressed renderer errors. */
export function createValidatedRenderer(): { renderer: WebGLRenderer; compatibility: boolean } {
  const profiles = [
    { antialias: true, powerPreference: 'high-performance' as const },
    { antialias: false, powerPreference: 'default' as const },
  ];
  for (const [index, profile] of profiles.entries()) {
    const canvas = document.createElement('canvas');
    let context: WebGL2RenderingContext | null = null;
    try {
      context = canvas.getContext('webgl2', { ...profile, alpha: true });
      if (!context || !hasUsableWebGL2Capabilities(context)) {
        release(context, canvas);
        continue;
      }
      // Choose a precision that the actual driver has just reported as usable.
      const high = context.getShaderPrecisionFormat(context.VERTEX_SHADER, context.HIGH_FLOAT)!.precision > 0
        && context.getShaderPrecisionFormat(context.FRAGMENT_SHADER, context.HIGH_FLOAT)!.precision > 0;
      const renderer = new WebGLRenderer({ ...profile, alpha: true, canvas, context, precision: high ? 'highp' : 'mediump' });
      return { renderer, compatibility: index === 1 };
    } catch {
      // Constructor exceptions still reach the real browser console if Three reports them.
      // Never conceal a failed renderer behind a synthetic "healthy" capability response.
      release(context, canvas);
    }
  }
  throw new Error('无法启动 3D 场景：两种图形配置均未提供有效的 WebGL 2。请开启图形加速或更新系统／浏览器后重试。');
}
