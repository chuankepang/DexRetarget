"""Small cleanup helpers shared by normal exit and partial initialization failure."""
import sys


def close_resource(resources, label, operation=None):
    close = resources.pop(label, None) or operation
    if close is not None:
        try:
            close()
        except Exception as exc:
            print(f'Cleanup {label} failed: {exc}', file=sys.stderr)
            return exc
    return None


def close_resources(resources):
    errors = []
    for label in reversed(list(resources)):
        error = close_resource(resources, label)
        if error is not None:
            errors.append((label, error))
    return errors
