// Injected before application scripts only for an explicit --ui-smoke-test run.
window.__EMULATION_UI_SMOKE__ = true;
window.__EMULATION_UI_DIAGNOSTICS__ = { consoleErrors: [], events: [] };
const originalConsoleError = console.error;
console.error = function (...args) {
  const errors = window.__EMULATION_UI_DIAGNOSTICS__.consoleErrors;
  if (errors.length < 20) errors.push(args.map(value => String(value)).join(" ").slice(0, 1500));
  return originalConsoleError.apply(console, args);
};
window.addEventListener("error", event => {
  const events = window.__EMULATION_UI_DIAGNOSTICS__.events;
  if (events.length < 20) events.push(String(event.message || "resource error").slice(0, 1500));
});
window.addEventListener("unhandledrejection", event => {
  const events = window.__EMULATION_UI_DIAGNOSTICS__.events;
  if (events.length < 20) events.push(String(event.reason).slice(0, 1500));
});
