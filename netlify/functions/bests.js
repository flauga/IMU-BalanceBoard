// GET /.netlify/functions/bests?mac=MG24-...
//
// Returns this board's BEST (highest) score for every level it has ever played,
// as a { "1": 1234, "2": 987, ... } map. Used by steadysteps.html so the
// per-level "Your best" numbers and level-unlock progress follow the physical
// board to ANY device — no login, no per-device localStorage dependence.
//
// Every level finish is already written as a Gameplay Session (see
// record-session.js); this reads them back and reduces to a per-level max.

const { SESSIONS, HttpError, findBoardRecordId, airtable, formulaQuote, reply } = require('./_airtable');

exports.handler = async (event) => {
  if (event.httpMethod !== 'GET') return reply(405, { error: 'Method not allowed' });

  const mac = ((event.queryStringParameters || {}).mac || '').trim();
  if (!mac) return reply(400, { error: 'Missing mac query param' });

  try {
    const recordId = await findBoardRecordId(mac);
    if (!recordId) return reply(200, { ok: true, bests: {} }); // unregistered board -> nothing yet

    // Pull this board's sessions, highest score first, and keep the first
    // (= max) score seen per level. maxRecords caps how far back we scan; a few
    // hundred is plenty to have every level's all-time best near the top.
    const formula = `{MAC ID}=${formulaQuote(mac)}`;
    const q = `/${SESSIONS.table}`
      + `?filterByFormula=${encodeURIComponent(formula)}`
      + `&sort%5B0%5D%5Bfield%5D=${SESSIONS.score}&sort%5B0%5D%5Bdirection%5D=desc`
      + `&returnFieldsByFieldId=true`
      + `&maxRecords=500`;

    const data  = await airtable(q);
    const bests = {};
    for (const r of (data.records || [])) {
      const level = String(r.fields[SESSIONS.level] || '').trim();
      const score = r.fields[SESSIONS.score];
      if (!level || typeof score !== 'number') continue;
      if (!(level in bests) || score > bests[level]) bests[level] = score; // sorted desc, so first wins
    }
    return reply(200, { ok: true, bests });
  } catch (e) {
    const status = e instanceof HttpError ? e.status : 500;
    return reply(status, { error: e.message });
  }
};
