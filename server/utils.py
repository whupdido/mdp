import struct, json, pickle

def recv_exact(conn, n: int):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

def send_json(conn, obj: dict):
    data = json.dumps(obj).encode("utf-8")
    conn.sendall(struct.pack(">I", len(data)))
    conn.sendall(data)

def recv_json(conn):
    hdr = recv_exact(conn, 4)
    if not hdr: return None
    (L,) = struct.unpack(">I", hdr)
    data = recv_exact(conn, L)
    if not data: return None
    return json.loads(data.decode("utf-8"))

def send_pickle(conn, arr) -> None:
    blob = pickle.dumps(arr, protocol=pickle.HIGHEST_PROTOCOL)
    conn.sendall(struct.pack(">Q", len(blob)))
    conn.sendall(blob)

def recv_pickle(conn):
    hdr = recv_exact(conn, 8)
    if not hdr: return None
    (L,) = struct.unpack(">Q", hdr)
    # Defensive bound to catch protocol mismatches (e.g. JSON framed as pickle).
    # Prevents OverflowError in recv when an invalid huge length is parsed.
    if L > 64 * 1024 * 1024:
        raise ValueError(f"Invalid pickle frame length: {L} bytes")
    blob = recv_exact(conn, L)
    if not blob: return None
    return pickle.loads(blob)
