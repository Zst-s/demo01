"""Bounded, integrity-checked records and atomic durable replacement."""
import hashlib
import json
import os
import tempfile
from pathlib import Path


def packed_body(body):
    data = json.dumps(body, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    if len(data) > 4_000_000:
        raise ValueError('RECORD_TOO_LARGE')
    return data


def read_record(path, versions=(2, 3, 4)):
    if os.path.getsize(path) > 4_100_000:
        raise ValueError('RECORD_TOO_LARGE')
    with open(path, encoding='utf-8') as stream:
        wrapper = json.load(stream)
    body = wrapper['body']
    if body['version'] not in versions or hashlib.sha256(packed_body(body)).hexdigest() != wrapper['sha256']:
        raise ValueError('INVALID_RECORD_INTEGRITY')
    return body


def atomic_write_record(path, body):
    wrapper = {'body': body, 'sha256': hashlib.sha256(packed_body(body)).hexdigest()}
    directory = os.path.dirname(os.path.abspath(path))
    fd, temporary = tempfile.mkstemp(prefix='.passage-', dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as out:
            os.fchmod(out.fileno(), 0o600)
            json.dump(wrapper, out, sort_keys=True, separators=(',', ':'), allow_nan=False)
            out.flush()
            os.fsync(out.fileno())
        os.replace(temporary, path)
        folder = os.open(directory, os.O_RDONLY)
        try:
            os.fsync(folder)
        finally:
            os.close(folder)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def backup_previous_record(path, next_epoch):
    """Keep exact legacy bytes, or the previous v4 map session, before replacement."""
    if not os.path.exists(path):
        return None
    body = read_record(path)
    if body['version'] == 4:
        if not isinstance(body.get('context'), dict):
            raise ValueError('INVALID_RECORD_CONTEXT')
        if body['context'].get('epoch') == next_epoch:
            return None
    data = Path(path).read_bytes()
    destination = str(path) + f'.v{body["version"]}-backup-' + hashlib.sha256(data).hexdigest()[:16]
    try:
        fd = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        if Path(destination).read_bytes() != data:
            raise ValueError('MIGRATION_BACKUP_CONFLICT')
        return destination
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        folder = os.open(os.path.dirname(os.path.abspath(path)), os.O_RDONLY)
        try:
            os.fsync(folder)
        finally:
            os.close(folder)
    except BaseException:
        os.unlink(destination)
        raise
    return destination
