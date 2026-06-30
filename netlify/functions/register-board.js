// POST /.netlify/functions/register-board
// Body: { mac, deviceModel?, manufactureDate?, status? }
//
// Upserts a Balance Boards row keyed on MAC ID, so re-registering the same
// physical board updates its row instead of creating a duplicate.

const { BOARDS, HttpError, upsertBoard, reply } = require('./_airtable');

const VALID_STATUS = ['Active', 'Inactive', 'Retired', 'Maintenance'];

exports.handler = async (event) => {
  if (event.httpMethod !== 'POST') return reply(405, { error: 'Method not allowed' });

  let payload;
  try { payload = JSON.parse(event.body || '{}'); }
  catch { return reply(400, { error: 'Invalid JSON body' }); }

  const mac = (payload.mac || '').trim();
  if (!mac) return reply(400, { error: 'Missing board MAC id' });

  // Only set fields that were supplied, so re-registration never blanks data.
  const fields = {};
  if (payload.deviceModel)     fields[BOARDS.deviceModel] = String(payload.deviceModel);
  if (payload.manufactureDate) fields[BOARDS.manufacture] = String(payload.manufactureDate); // YYYY-MM-DD
  if (payload.status) {
    if (!VALID_STATUS.includes(payload.status)) return reply(400, { error: 'Invalid status' });
    fields[BOARDS.status] = payload.status;
  }

  try {
    const { recordId, created } = await upsertBoard(mac, fields);
    return reply(200, { ok: true, mac, recordId, created });
  } catch (e) {
    const status = e instanceof HttpError ? e.status : 500;
    return reply(status, { error: e.message });
  }
};
