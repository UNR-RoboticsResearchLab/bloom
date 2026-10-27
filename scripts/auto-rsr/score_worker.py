"""Run the pinned pure edit-search functions with a bounded CPU/memory budget.

Only these two original function definitions are loaded; the worker does not
import WhisperX or initialize another CUDA model.
"""
import ast
import heapq
import json
from pathlib import Path
import resource
import sys


def load_functions(source):
    tree = ast.parse(Path(source).read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                 and n.name in ('score_rsr_errors', 'score_rsr')]
    if len(functions) != 2:
        raise RuntimeError('Pinned AutoRSR scoring functions are missing.')
    namespace = {'heapq': heapq}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace


if __name__ == '__main__':
    resource.setrlimit(resource.RLIMIT_AS, (512 * 1024 * 1024, 512 * 1024 * 1024))
    resource.setrlimit(resource.RLIMIT_CPU, (3, 3))
    scorer = load_functions(sys.argv[1])
    payload = json.load(sys.stdin)
    edits = scorer['score_rsr_errors'](payload['response'], payload['target'])
    errors = sum(len(v) for v in edits.values())
    print(json.dumps(dict(editScript=edits, errors=errors, score=scorer['score_rsr']([errors]))))
