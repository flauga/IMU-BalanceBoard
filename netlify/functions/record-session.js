// POST /.netlify/functions/record-session
// Body: { mac, level, score }
//
// Records one Gameplay Sessions row linked to the board. If the board has not
// been registered yet, we create a minimal Balance Boards row first so a
// session is never dropped on an unregistered board.

const { SESSIONS, HttpError, findBoardRecordId, upsertBoard, airtable, reply } = require('./_airtable');

const VALID_LEVELS = ['1', '2', '3', '4', '5'];

exports.handler = async (event) => {
  if (event.httpMethod !== 'POST') return reply(405, { error: 'Method not allowed' });

  let payload;
  try { payload = JSON.parse(event.body || '{}'); }
  catch { return reply(400, { error: 'Invalid JSON body' }); }

  const mac   = (payload.mac || '').trim();
  const level = String(payload.level == null ? '' : payload.level).trim();
  const score = Math.round(Number(payload.score));

  if (!mac) return reply(400, { error: 'Missing board MAC id' });
  if (!VALID_LEVELS.includes(level)) return reply(400, { error: 'Game Level must be "1".."5"' });
  if (!Number.isFinite(score)) return reply(400, { error: 'Score must be a number' });

  try {
    // Resolve the board's record id; auto-create a stub row if it's unknown.
    let recordId = await findBoardRecordId(mac);
    if (!recordId) ({ recordId } = await upsertBoard(mac, {}));

    const body = {
      records: [{
        fields: {
          [SESSIONS.level]:   level,        // single select "1".."5"
          [SESSIONS.score]:   score,        // integer
          [SESSIONS.macLink]: [recordId],   // link field expects record ids
        },
      }],
      typecast: true,
    };
    const data = await airtable(`/${SESSIONS.table}`, { method: 'POST', body });
    const rec = data.records && data.records[0];
    return reply(200, { ok: true, id: rec && rec.id });
  } catch (e) {
    const status = e instanceof HttpError ? e.status : 500;
    return reply(status, { error: e.message });
  }
};
