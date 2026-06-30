// GET /.netlify/functions/leaderboard?mac=MG24-...&limit=10&level=3
//
// Returns the top gameplay sessions for one board, highest score first. Used by
// steadysteps.html to render a per-board leaderboard (replacing Firebase).

const { SESSIONS, HttpError, findBoardRecordId, airtable, formulaQuote, reply } = require('./_airtable');

exports.handler = async (event) => {
  if (event.httpMethod !== 'GET') return reply(405, { error: 'Method not allowed' });

  const params = event.queryStringParameters || {};
  const mac    = (params.mac || '').trim();
  const level  = (params.level || '').trim();
  const limit  = Math.min(Math.max(parseInt(params.limit, 10) || 10, 1), 100);

  if (!mac) return reply(400, { error: 'Missing mac query param' });

  try {
    const recordId = await findBoardRecordId(mac);
    if (!recordId) return reply(200, { ok: true, entries: [] }); // unregistered board -> empty board

    // Filter by the linked board. The link field renders as the board's primary
    // field (MAC ID) in a formula, so match on the MAC string. Optionally also
    // constrain to one level.
    const clauses = [`{MAC ID}=${formulaQuote(mac)}`];
    if (level) clauses.push(`{Game Level}=${formulaQuote(level)}`);
    const formula = clauses.length > 1 ? `AND(${clauses.join(',')})` : clauses[0];

    const q = `/${SESSIONS.table}`
      + `?filterByFormula=${encodeURIComponent(formula)}`
      + `&sort%5B0%5D%5Bfield%5D=${SESSIONS.score}&sort%5B0%5D%5Bdirection%5D=desc`
      + `&returnFieldsByFieldId=true`   // so r.fields is keyed by field id, matching the constants below
      + `&maxRecords=${limit}`;

    const data = await airtable(q);
    const entries = (data.records || []).map(r => ({
      score: r.fields[SESSIONS.score] || 0,
      level: r.fields[SESSIONS.level] || '',
      t:     r.createdTime ? Date.parse(r.createdTime) : 0,
    }));
    return reply(200, { ok: true, entries });
  } catch (e) {
    const status = e instanceof HttpError ? e.status : 500;
    return reply(status, { error: e.message });
  }
};
