import type { SourceDiagnostic } from './types';

/** Pending template fields must not exhaust the separate budget for syntax errors. */
export function templateParameterReporter(diagnostics: SourceDiagnostic[]): (diagnostic: SourceDiagnostic) => void {
  let count = 0;
  let summary: SourceDiagnostic | undefined;
  const detailLimit = 32;
  return diagnostic => {
    count++;
    if (count <= detailLimit) { diagnostics.push(diagnostic); return; }
    if (!summary) {
      summary = { ...diagnostic };
      diagnostics.push(summary);
    }
    summary.message = `另有 ${count - detailLimit} 处模板参数待配置，请检查全部占位符；当前模板不能直接用于控制器。`;
  };
}
