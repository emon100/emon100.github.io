import pages from './i18n-pages.mjs';
import research from './i18n-research.mjs?v=20261022-recovery';

const supported = ['zh-CN', 'en'];
const storageKey = 'roadster-language';
const dictionary = { ...research, ...pages };
function initialLocale() {
  const query = typeof location === 'object' ? new URLSearchParams(location.search).get('lang') : null;
  if (supported.includes(query)) return query;
  try {
    const saved = globalThis.localStorage?.getItem(storageKey);
    if (supported.includes(saved)) return saved;
  } catch { /* Storage may be blocked by browser policy. */ }
  return /^zh\b/i.test(globalThis.navigator?.language || '') ? 'zh-CN' : 'en';
}
const locale = initialLocale();
export const getLocale = () => locale;
export function t(zh, en, values = {}) {
  const text = locale === 'en' ? en : zh;
  return String(text ?? '').replace(/\{(\w+)\}/g, (match, key) => Object.hasOwn(values, key) ? String(values[key]) : match);
}
export function translateText(source) {
  if (locale !== 'en' || typeof source !== 'string') return source;
  const key = source.trim();
  return Object.hasOwn(dictionary, key) ? source.replace(key, () => dictionary[key]) : source;
}
function initialize() {
  document.documentElement.lang = locale;
  try { globalThis.localStorage?.setItem(storageKey, locale); } catch { /* Storage is optional. */ }
  if (locale === 'en') {
    const walker = document.createTreeWalker(document.documentElement, NodeFilter.SHOW_TEXT, {
      acceptNode: node => node.parentElement?.closest('script,style,code,pre') ? NodeFilter.FILTER_REJECT : NodeFilter.FILTER_ACCEPT,
    });
    const nodes = [];
    while (walker.nextNode()) nodes.push(walker.currentNode);
    for (const node of nodes) node.nodeValue = translateText(node.nodeValue);
    for (const element of document.querySelectorAll('*')) {
      if (element.closest('script,style,code,pre')) continue;
      for (const attribute of ['title', 'aria-label', 'alt', 'placeholder']) {
        if (element.hasAttribute(attribute)) element.setAttribute(attribute, translateText(element.getAttribute(attribute)));
      }
      if (element.hasAttribute('data-i18n-en')) element.textContent = element.getAttribute('data-i18n-en');
      if (element.matches('meta[name="description"]')) element.content = translateText(element.content);
    }
  }
  const label = document.createElement('label');
  label.className = 'language-selector';
  const name = document.createElement('span');
  name.className = 'language-selector-name';
  name.textContent = t('语言', 'Language');
  const select = document.createElement('select');
  select.setAttribute('aria-label', t('网站语言', 'Site language'));
  select.title = t('切换语言会重新加载当前页面', 'Switching language reloads this page');
  for (const [value, text] of [['zh-CN', '简体中文'], ['en', 'English']]) {
    const option = document.createElement('option');
    option.value = value;
    option.textContent = text;
    option.selected = value === locale;
    select.append(option);
  }
  select.addEventListener('change', () => {
    const next = select.value;
    if (!supported.includes(next)) return;
    try { globalThis.localStorage?.setItem(storageKey, next); } catch { /* URL remains the fallback. */ }
    const url = new URL(location.href);
    url.searchParams.set('lang', next);
    location.assign(url.href);
  });
  label.append(name, select);
  const header = document.querySelector('.site-header');
  if (header) header.append(label);
  else {
    label.classList.add('language-selector-standalone');
    document.body.prepend(label);
  }
  // Preserve the explicit language when opening another local page or sharing its link.
  for (const link of document.querySelectorAll('a[href]')) {
    const raw = link.getAttribute('href');
    if (!raw || raw.startsWith('#')) continue;
    const url = new URL(raw, location.href);
    if (url.origin === location.origin && /\.html$/.test(url.pathname)) {
      url.searchParams.set('lang', locale);
      link.setAttribute('href', url.pathname + url.search + url.hash);
    }
  }
}
if (typeof document === 'object') {
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', initialize, { once: true });
  else initialize();
}
