import type { Completion } from '@codemirror/autocomplete';
import { EditorView, ViewPlugin } from '@codemirror/view';

type EditorIcon = 'close' | 'chevron-down' | 'chevron-right' | 'more' | 'function' | 'class' | 'variable' | 'constant' | 'property' | 'keyword' | 'text';
const paths: Partial<Record<EditorIcon, string>> = {
  close: 'M6 6l12 12M18 6 6 18',
  'chevron-down': 'm6 9 6 6 6-6',
  'chevron-right': 'm9 6 6 6-6 6',
  function: 'm8 6-6 6 6 6m8-12 6 6-6 6m-3-14-2 16',
  class: 'm12 2 9 5v10l-9 5-9-5V7Zm0 10 9-5M3 7l9 5m0 0v10',
  constant: 'm12 3 9 9-9 9-9-9Zm-4 9h8',
  property: 'M4 4h16v16H4Zm0 5h16M9 9v11',
  keyword: 'm14 10 7-7m-3 3 3 3m-6 0 3 3',
  text: 'M4 5h16M4 10h16M4 15h10M4 20h7',
};

/** CodeMirror owns these nodes, so keep its icons independent of React roots. */
function svgIcon(name: EditorIcon, owner: Document = document): SVGSVGElement {
  const namespace = 'http://www.w3.org/2000/svg';
  const svg = owner.createElementNS(namespace, 'svg');
  for (const [key, value] of Object.entries({ viewBox: '0 0 24 24', width: '14', height: '14', fill: 'none', stroke: 'currentColor', 'stroke-width': '1.7', 'stroke-linecap': 'round', 'stroke-linejoin': 'round', 'aria-hidden': 'true', focusable: 'false', class: 'robot-editor-icon', 'data-icon': name })) svg.setAttribute(key, value);
  if (paths[name]) {
    const path = owner.createElementNS(namespace, 'path');
    path.setAttribute('d', paths[name]!); svg.append(path);
  }
  const circles = name === 'more' ? [[5, 12, 1.5], [12, 12, 1.5], [19, 12, 1.5]]
    : name === 'variable' ? [[12, 12, 8], [12, 12, 2]]
    : name === 'keyword' ? [[8, 16, 5]] : [];
  for (const [cx, cy, radius] of circles) {
    const circle = owner.createElementNS(namespace, 'circle');
    circle.setAttribute('cx', String(cx)); circle.setAttribute('cy', String(cy)); circle.setAttribute('r', String(radius));
    if (name === 'more') circle.setAttribute('fill', 'currentColor');
    svg.append(circle);
  }
  return svg;
}

export function completionIcon(completion: Completion): HTMLElement {
  const type = completion.type?.split(' ')[0];
  const name: EditorIcon = type === 'function' || type === 'method' ? 'function'
    : type === 'class' || type === 'type' || type === 'interface' || type === 'namespace' ? 'class'
    : type === 'variable' ? 'variable' : type === 'constant' || type === 'enum' ? 'constant'
    : type === 'property' ? 'property' : type === 'keyword' ? 'keyword' : 'text';
  const wrapper = document.createElement('span');
  wrapper.className = 'robot-completion-icon';
  wrapper.setAttribute('aria-hidden', 'true');
  wrapper.append(svgIcon(name));
  return wrapper;
}

export function foldMarker(open: boolean): HTMLElement {
  const marker = document.createElement('span');
  marker.className = 'robot-fold-marker';
  marker.title = open ? '折叠代码' : '展开代码';
  marker.append(svgIcon(open ? 'chevron-down' : 'chevron-right'));
  return marker;
}

export function foldPlaceholder(view: EditorView, onclick: (event: MouseEvent) => void): HTMLElement {
  const placeholder = view.dom.ownerDocument.createElement('button');
  placeholder.type = 'button';
  placeholder.className = 'cm-foldPlaceholder';
  placeholder.setAttribute('aria-label', view.state.phrase('folded code'));
  placeholder.title = view.state.phrase('unfold');
  placeholder.append(svgIcon('more', view.dom.ownerDocument));
  placeholder.onclick = onclick;
  return placeholder;
}

const panelCloseSelector = '.cm-search button[name="close"], .cm-panel-lint button[name="close"]';
function decoratePanelClose(root: Element) {
  const buttons = root.matches(panelCloseSelector) ? [root] : root.querySelectorAll(panelCloseSelector);
  for (const button of buttons) {
    if (button.querySelector('svg.robot-editor-icon')) continue;
    // Keep CodeMirror's button, accessible name and click handler intact.
    button.replaceChildren(svgIcon('close', button.ownerDocument));
  }
}

/** Panels are created lazily by CodeMirror; only inspect newly inserted nodes. */
export const editorPanelIcons = ViewPlugin.fromClass(class {
  private observer: MutationObserver;
  constructor(view: EditorView) {
    decoratePanelClose(view.dom);
    this.observer = new MutationObserver(records => {
      for (const record of records) for (const node of record.addedNodes) {
        if (node.nodeType === Node.ELEMENT_NODE) decoratePanelClose(node as Element);
      }
    });
    this.observer.observe(view.dom, { childList: true, subtree: true });
  }
  destroy() { this.observer.disconnect(); }
});
