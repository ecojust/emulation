// Static imports make the inline worker self-contained: no CDN or custom-protocol
// module fetching is needed in the installed macOS/Windows WebView.
import { createRapidLanguage, rapidLanguage } from './languages/rapid';
import { fanucLanguage } from './languages/fanuc';
import { informLanguage } from './languages/inform';
import { krlLanguage } from './languages/krl';
import { snapshotAnalysisRequest, type AnalysisMessage, type AnalysisReply, type ProjectAnalysis } from './analysis-protocol';
import type { RobotLanguageDefinition } from './types';
const languages: Record<string, RobotLanguageDefinition> = {
  'rapid-rw7': rapidLanguage, 'rapid-rw6': createRapidLanguage('6.16'),
  'fanuc-r30ib': fanucLanguage, 'inform-yrc1000': informLanguage, 'krl-kss8': krlLanguage,
};
self.onmessage = (event: MessageEvent<AnalysisMessage>) => {
  const request = event.data;
  const reply: AnalysisReply = { requestId: request?.requestId, modelId: request?.modelId, sourceRevision: request?.sourceRevision, dialectId: request?.dialectId };
  try {
    if (typeof request?.requestId !== 'string' || !request.requestId || request.requestId.length > 128) throw new Error('语法分析请求标识无效。');
    const snapshot = snapshotAnalysisRequest(request);
    const language = Object.hasOwn(languages, snapshot.dialectId) ? languages[snapshot.dialectId] : null;
    if (!language) throw new Error('语法分析请求无效。');
    const byFile: ProjectAnalysis['byFile'] = Object.create(null);
    for (const file of snapshot.files) {
      byFile[file.id] = language.analyze(file.text, file.name);
    }
    reply.result = { byFile };
  } catch (cause) { reply.error = cause instanceof Error ? cause.message : '语法分析未能完成。'; }
  self.postMessage(reply);
};
