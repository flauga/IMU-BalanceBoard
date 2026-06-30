// Shared Airtable helper for the Netlify Functions proxy.
//
// The Airtable Personal Access Token lives ONLY here (read from the
// AIRTABLE_PAT env var on the server) — it never reaches the browser. Every
// dashboard talks to the functions in this folder, which talk to Airtable.
//
// We address tables and fields by their stable IDs (tbl.../fld...) rather than
// names, so renaming a field in the Airtable UI never breaks these calls.

const BASE_ID = process.env.AIRTABLE_BASE_ID || 'appCoGPA1LTG7TO6g';
const PAT     = process.env.AIRTABLE_PAT;

// Balance Boards table + field ids.
const BOARDS = {
  table:       'tblduFPUhD3Jucr7F',
  macId:       'fld20KP9Py0jAiqwi', // text — registration key, e.g. MG24-AABBCCDDEEFF
  deviceModel: 'fldjuAVEMxTlX0bSa', // text
  manufacture: 'fldQL8EuG2yMeo1W7', // date YYYY-MM-DD
  status:      'fld0Ti4eCPNQatbkA', // single select: Active/Inactive/Retired/Maintenance
};

// Gameplay Sessions table + field ids.
const SESSIONS = {
  table:   'tbl2B31bN2MrPiSn1',
  level:   'fldFkowtuBGNRq88k', // single select "1".."5"
  score:   'fldSC7W4MVKQlx4fL', // number (integer)
  macLink: 'fld3MqKFwnlYcWWTB', // link -> Balance Boards (array of record IDs)
};

const API = 'https://api.airtable.com/v0';

// Throw a typed error carrying an HTTP status so handlers can relay it.
class HttpError extends Error {
  constructor(status, message) { super(message); this.status = status; }
}

function requireToken() {
  if (!PAT) throw new HttpError(500, 'Server is missing AIRTABLE_PAT — set it in Netlify env vars / .env.');
}

// Thin fetch wrapper around the Airtable REST API. `path` is appended to the
// base URL (e.g. `/${BOARDS.table}`). Returns parsed JSON; throws HttpError on
// a non-2xx so the caller can pass the status straight back to the browser.
async function airtable(path, { method = 'GET', body } = {}) {
  requireToken();
  const res = await fetch(`${API}/${BASE_ID}${path}`, {
    method,
    headers: {
      Authorization: `Bearer ${PAT}`,
      'Content-Type': 'application/json',
    },
    body: body ? JSON.stringify(body) : undefined,
  });
  const text = await res.text();
  let json = null;
  try { json = text ? JSON.parse(text) : null; } catch { /* non-JSON error body */ }
  if (!res.ok) {
    const detail = (json && json.error && (json.error.message || json.error.type)) || text || res.statusText;
    throw new HttpError(res.status, `Airtable ${res.status}: ${detail}`);
  }
  return json;
}

// Escape a value for use inside an Airtable formula string literal.
function formulaQuote(v) { return `'${String(v).replace(/'/g, "\\'")}'`; }

// Module-scope MAC -> record-id cache. Functions are short-lived, but a warm
// container reuses this across invocations, saving a lookup per session POST.
const macCache = new Map();

// Resolve a board MAC to its Balance Boards record id. Returns null if no row
// exists (the caller decides whether to create one).
async function findBoardRecordId(mac) {
  if (macCache.has(mac)) return macCache.get(mac);
  const formula = `{MAC ID}=${formulaQuote(mac)}`;
  const q = `/${BOARDS.table}?maxRecords=1&filterByFormula=${encodeURIComponent(formula)}`;
  const data = await airtable(q);
  const id = (data && data.records && data.records[0] && data.records[0].id) || null;
  if (id) macCache.set(mac, id);
  return id;
}

// Upsert a Balance Boards row keyed on MAC ID. `fields` is keyed by field id.
// Returns { recordId, created }.
async function upsertBoard(mac, fields) {
  const body = {
    performUpsert: { fieldsToMergeOn: [BOARDS.macId] },
    records: [{ fields: { [BOARDS.macId]: mac, ...fields } }],
    typecast: true, // let single-select values (Board Status) match by name
  };
  const data = await airtable(`/${BOARDS.table}`, { method: 'PATCH', body });
  const rec = data.records && data.records[0];
  const recordId = rec ? rec.id : null;
  const created = !!(data.createdRecords && data.createdRecords.includes(recordId));
  if (recordId) macCache.set(mac, recordId);
  return { recordId, created };
}

// Standard JSON response helper for Netlify Functions.
function reply(statusCode, obj) {
  return {
    statusCode,
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(obj),
  };
}

module.exports = {
  BASE_ID, BOARDS, SESSIONS,
  HttpError, airtable, formulaQuote,
  findBoardRecordId, upsertBoard, reply,
};
