"""Check owned Python syntax, module boundaries/cycles, and local Markdown links.

No imports of robot/camera code, hardware calls, network checks or third-party
analysis dependencies. Generated measurements and vendored manuals are excluded.
"""
from __future__ import annotations
import argparse
import ast
import json
import re
from pathlib import Path
from urllib.parse import unquote

ROOT=Path(__file__).resolve().parents[1]
EXCLUDED={'.git','.agents','.aws','.codex','__pycache__','.pytest_cache','assets','sdk','third_party','camera_tests','raw','lerobot','build','logs'}
# Official SDK checkouts are upstream reference material, not project-owned code.
EXCLUDED.update({'xCoreSDK-CPP', 'xCoreSDK-Python'})
COMPAT={'dexretarget/dataset/__main__.py','dexretarget/dataset/runtime.py','dexretarget/dataset/cameras.py','dexretarget/dataset/simulation.py'}


def owned_files(suffix):
    return sorted(p for p in ROOT.rglob('*'+suffix) if not (set(p.relative_to(ROOT).parts)&EXCLUDED) and not any('.egg-info' in part for part in p.parts))


def module_name(path):
    parts=list(path.relative_to(ROOT).with_suffix('').parts)
    if parts[-1]=='__init__':parts.pop()
    return '.'.join(parts)


def imports(tree,module,is_package):
    package=module.split('.') if is_package else module.split('.')[:-1]
    values=set()
    for node in ast.walk(tree):
        if isinstance(node,ast.Import):values.update(a.name for a in node.names)
        elif isinstance(node,ast.ImportFrom):
            prefix=package[:len(package)-node.level+1] if node.level else []
            prefix += node.module.split('.') if node.module else []
            values.add('.'.join(prefix))
            values.update('.'.join([*prefix,a.name]) for a in node.names if a.name!='*')
    return values


def python_checks():
    errors=[];graph={};count=0
    for path in owned_files('.py'):
        relative=path.relative_to(ROOT).as_posix()
        try:tree=ast.parse(path.read_text())
        except SyntaxError as error:
            errors.append(f'{relative}: {error}');continue
        count+=1;module=module_name(path);deps=imports(tree,module,path.name=='__init__.py')
        if relative.startswith('dexretarget/teleop/'):
            banned=('dexretarget.dataset','dexretarget.app','dexretarget.camera','integrations')
        elif relative.startswith('dexretarget/camera/'):
            banned=('dexretarget.dataset','dexretarget.app','dexretarget.teleop','rokae','integrations')
        elif relative.startswith('dexretarget/dataset/') and relative not in COMPAT:
            banned=('dexretarget.app','dexretarget.teleop','dexretarget.camera','integrations','rokae')
        else:banned=()
        for dep in deps:
            if any(dep==p or dep.startswith(p+'.') for p in banned):errors.append(f'Forbidden dependency {module} -> {dep}')
        if relative.startswith(('dexretarget/','dexretarget/dataset/')) and relative not in COMPAT:
            graph[module]=deps
    graph={m:{d for d in deps if d in graph} for m,deps in graph.items()}
    active=[];done=set()
    def visit(module):
        if module in active:
            errors.append('Dependency cycle: '+' -> '.join(active[active.index(module):]+[module]));return
        if module in done:return
        active.append(module)
        for dep in sorted(graph[module]):visit(dep)
        active.pop();done.add(module)
    for module in sorted(graph):visit(module)
    return errors,count,len(graph)


def anchors(path):
    result=set(re.findall(r'<a\s+(?:id|name)=["\']([^"\']+)',path.read_text()))
    seen={};fence=False
    for line in path.read_text().splitlines():
        if line.startswith('```'):fence=not fence;continue
        if fence:continue
        match=re.match(r'^#{1,6}\s+(.+?)\s*#*$',line)
        if match:
            slug=re.sub(r'[^\w\- ]','',match[1].lower()).replace(' ','-')
            number=seen.get(slug,0);seen[slug]=number+1
            result.add(slug+(f'-{number}' if number else ''))
    return result


def markdown_checks():
    errors=[];links=0;optional=0
    for path in owned_files('.md'):
        text=path.read_text();fenced=False;prose=[]
        for line in text.splitlines():
            if line.startswith('```'):fenced=not fenced
            elif not fenced:prose.append(line)
        if fenced:errors.append(f'Unclosed Markdown fence: {path.relative_to(ROOT)}')
        for value in re.findall(r'\[[^\]]+\]\(([^)]+)\)', '\n'.join(prose)):
            value=value.strip().strip('<>')
            if re.match(r'^[a-zA-Z]+:',value):continue
            file,_,fragment=value.partition('#')
            target=(path.parent/unquote(file)).resolve() if file else path
            try:relative=target.relative_to(ROOT)
            except ValueError:relative=target
            if str(relative).startswith(('data/camera_tests/','data/raw/','data/lerobot/')):
                optional+=1;continue
            links+=1
            if not target.exists():errors.append(f'{path.relative_to(ROOT)}: missing {value}')
            elif fragment and target.suffix=='.md' and unquote(fragment) not in anchors(target):
                errors.append(f'{path.relative_to(ROOT)}: missing anchor {value}')
    return errors,links,optional


def check():
    errors,files,modules=python_checks()
    md_errors,links,optional=markdown_checks()
    return {'python_files':files,'canonical_modules':modules,'local_markdown_links':links,
            'optional_generated_links':optional,'errors':errors+md_errors,
            'scope':'Owned source/docs only; no external URL or hardware validation.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    report=check();text=json.dumps(report,ensure_ascii=False,indent=2)
    print(text)
    if args.output:args.output.write_text(text+'\n')
    return bool(report['errors'])


if __name__=='__main__':raise SystemExit(main())
