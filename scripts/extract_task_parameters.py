#!/usr/bin/env python3
"""Extract user-selected 95%-ECDF CPU budgets from sealed caches; never simulate."""
from collections import defaultdict, deque
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, FancyArrowPatch

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'docs/evidence'
FIGURES = ROOT / 'docs/figures/extracted-parameters'
PERIODS = {'control_iteration': .05, 'local_costmap_update': .2,
           'global_costmap_update': 1., 'velocity_smoothing_tick': .05, 'bt_tick': .01}
NAMES = {'control_iteration': 'Control / MPPI', 'local_costmap_update': 'Local map',
         'global_costmap_update': 'Global map', 'velocity_smoothing_tick': 'Velocity timer',
         'bt_tick': 'BT tick', 'planning_request': 'Planning action',
         'amcl_scan_callback': 'AMCL scan', 'mppi_noise_generation': 'Noise helper',
         'velocity_command_callback': 'Command input', 'collision_check': 'Collision check',
         'controller_path_install': 'Path install'}
ORDER = list(NAMES)
IDS = {k: ('tau' + str(list(PERIODS).index(k) + 1) if k in PERIODS
           else 'J' + str([x for x in ORDER if x not in PERIODS].index(k) + 1)) for k in ORDER}


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def number(x):
    return '—' if x is None else f'{x:.8f}'.rstrip('0').rstrip('.')


def ecdf_budget(values):
    """The exact generalized inverse of the empirical CDF, without interpolation."""
    n = len(values)
    rank = (95 * n + 99) // 100
    budget = float(np.partition(values, rank - 1)[rank - 1])
    below = int(np.count_nonzero(values < budget))
    at = int(np.count_nonzero(values <= budget))
    assert below * 100 < 95 * n <= at * 100
    return {'C_model_s': budget, 'jobs': n, 'order_statistic_rank_1_based': rank,
            'cdf_below_C': below / n, 'cdf_at_C': at / n,
            'exceeding_jobs': n - at, 'exceedance_fraction': (n - at) / n}


def acyclic(nodes, edges):
    indegree = {k: 0 for k in nodes}
    adjacency = defaultdict(list)
    for e in edges:
        assert e['from'] in nodes and e['to'] in nodes and e['from'] != e['to']
        indegree[e['to']] += 1
        adjacency[e['from']].append(e['to'])
    ready = deque(k for k, v in indegree.items() if v == 0)
    count = 0
    while ready:
        v = ready.popleft()
        count += 1
        for child in adjacency[v]:
            indegree[child] -= 1
            if indegree[child] == 0:
                ready.append(child)
    assert count == len(nodes), 'Dependency graph contains a cycle'


def node_label(task, index, parameters):
    t = parameters[task]
    ident = t['task_id']
    math_id = (r'$\tau_' + ident[3:] + '[' + index + ']$' if ident.startswith('tau')
               else r'$J_' + ident[1:] + '[' + index + ']$')
    label = math_id + '\n' + NAMES[task] + '\nC = ' + number(t['C_model_s']) + ' s'
    if task in PERIODS:
        label += '\nT = ' + number(t['T_nominal_s']) + ' s'
    return label


def draw_graph(name, nodes, edges, positions, parameters, title, caption, figsize=(12, 11)):
    acyclic(nodes, edges)
    fig, ax = plt.subplots(figsize=figsize)
    patches = {}
    for key, v in nodes.items():
        x, y = positions[key]
        color = '#e6effa' if v['task'] in PERIODS else '#e8f4e9'
        patch = Ellipse((x, y), width=2.65, height=1.28,
                        facecolor=color, edgecolor='#243c55', linewidth=1.2, zorder=3)
        ax.add_patch(patch)
        patches[key] = patch
        ax.text(x, y, node_label(v['task'], v['index'], parameters),
                ha='center', va='center', fontsize=9.6, linespacing=1.15, zorder=4)
    for e in edges:
        ax.add_patch(FancyArrowPatch(positions[e['from']], positions[e['to']],
                                    patchA=patches[e['from']], patchB=patches[e['to']],
                                    arrowstyle='-|>', mutation_scale=15, linewidth=1.3,
                                    color='#334d67', connectionstyle=e.get('curve', 'arc3,rad=0'), zorder=2))
    xs, ys = zip(*positions.values())
    ax.set_xlim(min(xs)-1.65, max(xs)+1.65)
    ax.set_ylim(min(ys)-1., max(ys)+1.)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title(title + '\nC: assumed CPU budget at 95% ECDF; T: nominal HOST period; all times in s',
                 fontsize=12, pad=17)
    fig.text(.5, .025, caption, ha='center', va='bottom', fontsize=9, linespacing=1.4)
    fig.tight_layout(rect=(0, .08, 1, 1))
    for ext in ['png', 'svg']:
        fig.savefig(FIGURES / (name + '.' + ext), dpi=175, bbox_inches='tight')
    plt.close(fig)
    # Keep an editable code-native graph as well as the scientific figure.
    lines = ['digraph G {', 'graph [rankdir=TB];',
             'node [shape=ellipse, fontname="DejaVu Sans"];']
    for key, v in nodes.items():
        t = parameters[v['task']]
        label = t['task_id']+'['+v['index']+']\n'+NAMES[v['task']]+'\nC = '+number(t['C_model_s'])+' s'
        if v['task'] in PERIODS:
            label += '\nT = '+number(t['T_nominal_s'])+' s'
        lines.append(json.dumps(key)+' [label='+json.dumps(label)+'];')
    for e in edges:
        lines.append(json.dumps(e['from'])+' -> '+json.dumps(e['to'])+';')
    (FIGURES / (name+'.dot')).write_text('\n'.join(lines+['}'])+'\n')


def scheduling_graph(parameters):
    definitions = {
        'bt': ('bt_tick','b'), 'global': ('global_costmap_update','g'),
        'plan': ('planning_request','a'), 'path': ('controller_path_install','p'),
        'local': ('local_costmap_update','l'), 'noise_old': ('mppi_noise_generation','n'),
        'control': ('control_iteration','k'), 'noise_next': ('mppi_noise_generation','n+1'),
        'command': ('velocity_command_callback','u'), 'smooth': ('velocity_smoothing_tick','h'),
        'collision': ('collision_check','x'), 'amcl': ('amcl_scan_callback','j')}
    nodes = {k: {'task': v[0], 'index': v[1]} for k,v in definitions.items()}
    raw_edges = [('amcl','global','selected map-to-odom TF state'), ('bt','plan','action request'), ('global','plan','selected committed global grid'),
                 ('plan','path','action path result'), ('path','control','selected installed path'),
                 ('local','control','selected committed local grid'),
                 ('noise_old','control','selected committed noise'),
                 ('control','noise_next','noise generation trigger'),
                 ('control','command','command message'), ('command','smooth','selected held command'),
                 ('smooth','collision','smoothed command message')]
    edges = [{'from': a, 'to': b, 'dependency': meaning,
              'evidence': 'installed-version source and configuration; particular job selectors are model inputs'}
             for a,b,meaning in raw_edges]
    positions = {'bt': (0,7.2), 'global': (4,7.2), 'plan': (2,5.4), 'path': (2,3.6),
                 'local': (6,5.4), 'noise_old': (10,5.4), 'control': (6,3.6),
                 'noise_next': (10,1.8), 'command': (6,1.8), 'smooth': (6,0),
                 'collision': (6,-1.8), 'amcl': (4,9.2)}
    draw_graph('book-style-task-dag', nodes, edges, positions, parameters,
               'Task/job DAG for the atomic result-delivery scheduling model',
               'Arrows are selected data/trigger dependencies, not observed CPU execution order.\n'
               'The TF edge is a source-backed model relation; exact producing TF job versions were not traced.\n'
               'Cached-state indices select an available producer job; they do not force a fresh update every period.')
    result = {'nodes': nodes, 'edges': edges, 'acyclic': True,
              'semantics': 'Chosen scheduling abstraction: atomic output delivery at modeled job completion. '
                           'Current ROS can publish/trigger before function return. This is not an extracted claim '
                           'of whole-function completion precedence. Repeated helper jobs are indexed to unroll feedback.',
              'isolated_nodes': [],
              'isolation_scope': 'These selected task families are connected. Disconnected representatives in the measured '
                                 'figure mean no matched edge in its window, not global independence.',
              'task_parameters': 'extracted-parameters.json'}
    write_json(EVIDENCE/'extracted-task-dag.json', result)
    return result


def measured_graph(parameters):
    phase = json.loads((EVIDENCE/'observed-phase-dag.json').read_text())
    trace = json.loads((ROOT/'artifacts/task-profiling'/phase['campaign']/phase['run']/'trace-graph-evidence.json').read_text())
    spans = {(x['pid'], x['id']): x for x in trace['spans']}
    def primary_owner(key):
        for _ in range(30):
            s = spans.get(key)
            if s is None:
                raise ValueError('Missing primary owner: '+str(key))
            if s['kind'] in NAMES:
                return key
            key = (s['pid'], s['parent'])
        raise ValueError('Parent chain too deep')
    def event_owner(name):
        v = phase['nodes'][name]
        return primary_owner((v['pid'], v['scope_id'] if v['event'].startswith('scope_') else v['parent']))
    roots = {event_owner(k) for k,v in phase['nodes'].items() if v['event']=='scope_begin'}
    families = {spans[k]['kind'] for k in roots}
    added = []
    for family in ORDER:
        if family not in families:
            candidates = [x for x in trace['spans'] if x['kind']==family]
            assert candidates
            chosen = min(candidates, key=lambda x:x['wall_start_ns'])
            key = (chosen['pid'],chosen['id']); roots.add(key); added.append(key)
    keys = {k: 'job_'+str(k[0])+'_'+str(k[1]) for k in roots}
    nodes = {}
    family_jobs = defaultdict(list)
    for k in roots:
        family_jobs[spans[k]['kind']].append(k)
    for family, jobs in family_jobs.items():
        for i,k in enumerate(sorted(jobs, key=lambda x:spans[x]['wall_start_ns'])):
            nodes[keys[k]] = {'task': family, 'index': str(i), 'pid': k[0], 'scope_id': k[1],
                             'unmatched_representative': k in added}
    edges = []
    for e in phase['edges']:
        if e['type'] in ['recorded_version_dependency','recorded_trigger_dependency']:
            a,b = event_owner(e['from']),event_owner(e['to'])
            if a!=b:
                edges.append({'from':keys[a], 'to':keys[b], 'type':e['type'], 'source_phase_edge':e})
        elif e['type']=='matched_message_dependency':
            inputs = [x for x in phase['edges'] if x['type']=='matched_callback_input' and x['from']==e['to']]
            assert len(inputs)==1
            a,b = event_owner(e['from']),event_owner(inputs[0]['to'])
            edges.append({'from':keys[a], 'to':keys[b], 'type':'matched_message_and_callback',
                          'source_phase_edges':[e,inputs[0]]})
    # No same-thread order edges, including edges between independent jobs on one worker.
    connected = [k for k,v in nodes.items() if not v['unmatched_representative']]
    byfamily = {f:sorted([k for k in connected if nodes[k]['task']==f],key=lambda x:nodes[x]['index']) for f in NAMES}
    positions = {}
    specs = {'local_costmap_update': [(7,8)], 'mppi_noise_generation': [(0,6),(7,4),(14,2)],
             'control_iteration': [(3.5,6),(10.5,4)], 'velocity_command_callback': [(3.5,4),(10.5,2)]}
    for f, coords in specs.items():
        assert len(byfamily[f])==len(coords)
        positions.update(zip(byfamily[f],coords))
    independent = sorted([k for k,v in nodes.items() if v['unmatched_representative']], key=lambda x:ORDER.index(nodes[x]['task']))
    for i,k in enumerate(independent):
        positions[k]=(2+i%3*5, -1.2-i//3*1.65)
    draw_graph('verified-job-dependencies', nodes, edges, positions, parameters,
               'Measured job dependencies with CPU-order edges removed',
               'Bottom isolated vertices: other task representatives with no matched edge in this selected window.\n'
               'Isolation does not establish global independence. Arrows identify observed producing/consuming phases;\n'
               'atomic completion semantics apply only when the scheduling-model convention is adopted.', figsize=(12,12))
    result = {'campaign':phase['campaign'],'run':phase['run'],'nodes':nodes,'edges':edges,'acyclic':True,
              'removed_same_thread_edges':sum(e['type']=='observed_same_thread_order' for e in phase['edges']),
              'scope':'Matched state, trigger and DDS/input dependencies only; grouped by primary computational job. '
                      'Edges originate/terminate at recorded phases, not necessarily at whole-job end/start.',
              'standalone_vertices':independent}
    write_json(EVIDENCE/'extracted-observed-job-dag.json', result)
    return result


def cdf_figure(parameters, arrays):
    fig, axes = plt.subplots(4,3,figsize=(12,11))
    for ax,task in zip(axes.flat,ORDER):
        a = np.sort(arrays[task]); n=len(a)
        indices=np.unique(np.linspace(0,n-1,min(n,2500),dtype=int))
        ax.step(a[indices], (indices+1)/n, where='post',color='#35668d',lw=1.2)
        c=parameters[task]['C_model_s']
        ax.axhline(.95,color='#a13333',ls='--',lw=.9)
        ax.axvline(c,color='#a13333',ls='--',lw=.9)
        ax.scatter([c],[parameters[task]['cdf_at_C']],s=18,color='#a13333',zorder=4)
        ax.set_xlim(0,float(np.quantile(a,.995,method='inverted_cdf'))*1.04)
        ax.set_ylim(0,1.02); ax.set_xlabel('CPU time (s)',fontsize=8); ax.set_ylabel('Empirical CDF',fontsize=8)
        ax.set_title(NAMES[task]+'\nC = '+number(c)+' s',fontsize=9)
        ax.tick_params(labelsize=7); ax.grid(alpha=.2)
        ax.ticklabel_format(axis='x',style='sci',scilimits=(-3,3))
    axes.flat[-1].axis('off')
    fig.suptitle('Exact 95%-ECDF CPU budgets from 57 complete missions\nCurves subsampled for display; right tail view ends near Q99.5; quantiles use every job',fontsize=12)
    fig.tight_layout(rect=(0,0,1,.95))
    for ext in ['png','svg']:
        fig.savefig(FIGURES/('cpu-cdf-95.'+ext),dpi=175,bbox_inches='tight')
    plt.close(fig)


def markdown_table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join('---' for _ in headers)+' |']+
                     ['| '+' | '.join(str(v) for v in row)+' |' for row in rows])


def render_report(data, parameters, model, observed):
    rows=[[p['task_id'],NAMES[k],p['class'],number(p['C_model_s']),number(p['T_nominal_s']),
           number(p['observed_mean_inter_entry_s']),str(p['jobs'])] for k,p in parameters.items()]
    headers=['ID','Primary work unit','Type','C / assumed WCET (s)','Nominal T (s)','Mean interval (s)','Jobs']
    notes=[
        'C is the inclusive thread-CPU budget of the named primary work unit. T and observed inter-entry use HOST wall time. Both are expressed in seconds; identical units do not make their clocks interchangeable.',
        'The user selects the 95%-ECDF point as the model WCET. It is an assumed simulation budget, not a certified bound on all executions. The CSV explicitly retains the fraction and count of jobs exceeding it.',
        'For periodic implementations, use the configured nominal T. For aperiodic streams, T stays unassigned: their measured average inter-entry interval is descriptive and is not a guaranteed period or sporadic minimum. Trace arrivals can drive the initial scheduler.',
        'The earlier report used NumPy linear-interpolation percentiles. This extraction instead computes the exact generalized inverse of the empirical CDF (nearest-rank order statistic). Both values are retained in the CSV to explain small numerical differences.',
        'Whole-loop CPU budgets, when measured, are exported separately. Inclusive parent and nested-child costs must not both be charged. A parent/child call relation is containment; it is not an edge saying that the parent must finish before its child starts.',
        'The book-style scheduling DAG adopts atomic output delivery at modeled completion. This matches a job-result delivery abstraction; current upstream ROS can publish/trigger before the enclosing function returns. The measured phase DAG remains the reference for those exact boundaries.',
        'Cached-state edges select the already available producer version; they do not force each control job to wait for a fresh map, path or command update. Noise feedback is unrolled as old noise -> current control -> next noise.',
        'The measured-job diagram removes all same-thread resource-order edges. An isolated representative means no exact dependency matched in this chosen trace window. It does not imply that its task family never interacts with other ROS components. No per-job TF version or exact global-grid producer is invented.',
        'Task-family relations for planning, installed path and command smoothing follow the installed-version source/configuration. Their precise producer/consumer job selectors are model inputs, not new trace facts. The AMCL-to-global-map TF relation is source-backed at task-family level; exact TF job-version mapping and other live TF reads remain in the pass-through framework.',
        'Scheduler development can start with these budgets, configured periodic releases and recorded aperiodic arrivals. Unlisted or unresolved infrastructure callbacks continue as pass-through background work in ROS. No invented C, T, deadline or priority is assigned to them, and they do not block the prototype.',
        'Deadlines remain unassigned. No priority is inferred from historical execution order. Processor/core choices and dispatch policy belong to the next scheduling stage. Simulation was not rerun for this extraction.'
    ]
    formula=r'''For task i, let c_(i,k) be each measured inclusive CPU cost in seconds, N_i the pooled job count and F_i the empirical CDF.

$$\widehat F_i(c)=\frac{1}{N_i}\sum_{k=1}^{N_i}\mathbf{1}[c_{i,k}\le c],\qquad C_i^{\mathrm{model}}=\inf\{c:\widehat F_i(c)\ge0.95\}=c_{i,(\lceil0.95N_i\rceil)}.$$

This is the selected assumed WCET for the model. The definition guarantees F_i(C_i^-) < 0.95 <= F_i(C_i), including ties. D_i remains unassigned.'''
    intro=f"Parameters extracted from {data['successful_laps']} complete, unchanged full-v2 missions. The source campaign remains sealed; no new simulation was launched. All time values in this report and its new diagrams are seconds."
    util=f"For the five periodic named work units, sum(C_i/T_i) = {data['periodic_named_budget_utilization']:.9f}. This accounts only for their selected CPU budgets; it excludes aperiodic work and other ROS/Gazebo/background demand and is not a schedulability result. The nominal hyperperiod is 1 s."
    dep_rows=[[e['from'],e['to'],e['dependency']] for e in model['edges']]
    md='# Extracted parameters — assumed WCET at 95% empirical CDF\n\n'+intro+'\n\n'+formula+'\n\n## Primary task parameters\n\n'+markdown_table(headers,rows)+'\n\n'+util
    md+='\n\n## Book-style scheduling DAG\n\n![Task/job scheduling DAG](figures/extracted-parameters/book-style-task-dag.png)\n\n'+model['semantics']+'\n\n'+markdown_table(['Producer job','Consumer job','Dependency'],dep_rows)
    md+='\n\n## Verified job dependencies\n\n![Measured job dependencies](figures/extracted-parameters/verified-job-dependencies.png)\n\n'+f"The clean finite graph has {len(observed['nodes'])} vertices and {len(observed['edges'])} dependency edges. It removes {observed['removed_same_thread_edges']} same-thread order edges. Its isolated representatives are not asserted to be globally independent."
    md+='\n\n## Empirical CDFs\n\n![CPU empirical CDFs](figures/extracted-parameters/cpu-cdf-95.png)\n\n## Modeling decisions and scheduler readiness\n\n'+'\n\n'.join(notes)
    md+='\n\n## Artifacts and source\n\nPrimary parameters: docs/evidence/extracted-parameters.csv and extracted-parameters.json. All measured work-unit budgets, including nested work: extracted-subjob-parameters.csv. Source: sealed full-v2 per-run analysis-cache.npz and final task-characterization-summary.json. Graph evidence: extracted-task-dag.json and extracted-observed-job-dag.json. Figures are editable SVG and DOT as well as PNG. The prior measurement report remains docs/task-model.en.pdf.\n\nNotation: Buttazzo (2011), Section 2.2.2, printed pp. 28–29 (PDF pp. 45–46). In the textbook a predecessor completes before its successor starts; atomic result delivery is the explicit convention needed to use that interpretation for the coarser job graph here. Source/configuration mappings are described in docs/task-model.en.md and docs/task-abstraction-study.fa.md.\n'
    (ROOT/'docs/extracted-parameters.en.md').write_text(md)
    sys.path.append('/mnt/c/Users/Asus/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/Lib/site-packages')
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak
    from reportlab.lib import colors
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib.pagesizes import A4
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from PIL import Image as PILImage
    import html
    fonts=Path('/usr/share/fonts/truetype/dejavu')
    pdfmetrics.registerFont(TTFont('ExtractSans',str(fonts/'DejaVuSans.ttf')))
    pdfmetrics.registerFont(TTFont('ExtractSansBold',str(fonts/'DejaVuSans-Bold.ttf')))
    pdfmetrics.registerFontFamily('ExtractSans',normal='ExtractSans',bold='ExtractSansBold',italic='ExtractSans',boldItalic='ExtractSansBold')
    styles=getSampleStyleSheet()
    styles.add(ParagraphStyle(name='ExtractBody',fontName='ExtractSans',fontSize=9,leading=12.5,spaceAfter=8))
    styles.add(ParagraphStyle(name='ExtractTitle',fontName='ExtractSansBold',fontSize=15,leading=19,spaceAfter=12))
    styles.add(ParagraphStyle(name='ExtractCell',fontName='ExtractSans',fontSize=7.3,leading=10,wordWrap='CJK'))
    def paragraph(text): return Paragraph(html.escape(text),styles['ExtractBody'])
    def heading(text): return Paragraph(html.escape(text),styles['ExtractTitle'])
    def pdf_table(head,body,widths):
        cells=[[Paragraph(html.escape(str(v)),styles['ExtractCell']) for v in row] for row in [head]+body]
        table=Table(cells,colWidths=widths,repeatRows=1,hAlign='LEFT')
        table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e8eff8')),('GRID',(0,0),(-1,-1),.35,colors.HexColor('#bccbdd')),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),5),('RIGHTPADDING',(0,0),(-1,-1),5),('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5)]))
        return table
    def figure(name):
        p=FIGURES/(name+'.png')
        with PILImage.open(p) as im: w,h=im.size
        width=min(495,600*w/h);return Image(str(p),width=width,height=width*h/w)
    story=[heading('Extracted parameters'),paragraph('Assumed WCET at the 95% empirical CDF point'),paragraph(intro),
           paragraph('F_i(c) = count(c_i,k <= c) / N_i; C_i(model) = inf{c : F_i(c) >= 0.95} = the ceil(0.95 N_i)-th ordered CPU sample.'),
           paragraph('This model WCET is the user-selected CPU budget; it is not a certified worst-case bound. All new table/diagram time values are in seconds.'),
           pdf_table(headers,rows,[29,91,29,82,53,70,53]),Spacer(1,12),paragraph(util),PageBreak(),
           heading('Book-style task/job scheduling DAG'),figure('book-style-task-dag'),paragraph(model['semantics']),PageBreak(),
           heading('Verified job dependencies'),figure('verified-job-dependencies'),paragraph(f"{len(observed['nodes'])} vertices, {len(observed['edges'])} dependency edges; {observed['removed_same_thread_edges']} CPU-order edges removed. Other task representatives are isolated only within this selected window."),PageBreak(),
           heading('Empirical CPU CDFs'),figure('cpu-cdf-95'),paragraph('The plotted curves are subsampled for readability. Every recorded job contributes to the exact nearest-rank quantile. Red lines mark the selected 95%-CDF budget.'),PageBreak(),
           heading('Dependency meanings'),pdf_table(['Producer','Consumer','Selected data / trigger'],dep_rows,[90,90,273]),Spacer(1,12),heading('Modeling decisions and scheduler readiness')]
    story += [paragraph(x) for x in notes]
    story += [paragraph('Nested and whole-loop budgets, per-task sample counts, exceedances and prior linear Q95 values are available in the CSV/JSON artifacts. Original measurements and raw CSVs remain unchanged.'),paragraph('Reference: Buttazzo (2011), Section 2.2.2, printed pp. 28-29 / PDF pp. 45-46. Source/configuration evidence is preserved in the prior English measurement report.')]
    def footer(canvas,doc):
        canvas.setFont('ExtractSans',7); canvas.drawString(42,27,'Extracted parameters | seconds | empirical 95%-CDF CPU model'); canvas.drawRightString(A4[0]-42,27,str(doc.page))
    SimpleDocTemplate(str(ROOT/'docs/extracted-parameters.en.pdf'),pagesize=A4,rightMargin=42,leftMargin=42,topMargin=38,bottomMargin=42,title='Extracted parameters - 95% empirical CDF').build(story,onFirstPage=footer,onLaterPages=footer)


def main():
    FIGURES.mkdir(parents=True,exist_ok=True)
    source=EVIDENCE/'task-characterization-summary.json'
    summary=json.loads(source.read_text())
    assert summary['status']=='FINAL' and summary['successful_analyzed_laps']==57
    buckets=defaultdict(list); loops=defaultdict(list)
    for trial in summary['trials']:
        run=ROOT/'artifacts/task-profiling'/trial['campaign']/trial['run']
        assert json.loads((run/'metadata.json').read_text())['passed']
        with np.load(run/'analysis-cache.npz') as cache:
            for task in summary['tasks']:
                key=task+'::cpu_inclusive_ms'
                if key in cache: buckets[task].append(cache[key]/1000.)
                key=task+'::full_iteration_cpu_ms'
                if key in cache: loops[task].append(cache[key]/1000.)
    arrays={k:np.concatenate(v) for k,v in buckets.items()}
    parameters={}; all_work=[]
    for task in summary['tasks']:
        values=arrays[task]; old=summary['tasks'][task]['metrics']; assert len(values)==old['cpu_inclusive_ms']['n']
        row={'task':task,'task_id':IDS.get(task,task),'class':'P' if task in PERIODS else 'A' if task in NAMES else 'nested_or_wait',
             'T_nominal_s':PERIODS.get(task),'C_mean_s':float(values.mean()),'C_std_s':float(values.std(ddof=1)),
             'prior_linear_Q95_s':old['cpu_inclusive_ms']['p95']/1000.,
             'observed_mean_inter_entry_s':old['inter_entry_wall_ms']['mean']/1000. if old.get('inter_entry_wall_ms',{}).get('mean') is not None else None,
             'whole_loop_C_model_s':ecdf_budget(np.concatenate(loops[task]))['C_model_s'] if loops.get(task) else None,
             'deadline_s':None,'priority':None,'granularity':'primary_named_inclusive' if task in NAMES else 'nested_inclusive_or_loop_wait_not_independent',
             'budget_status':'USER_ASSUMED_WCET_Q95_ECDF','cpu_clock':'HOST_THREAD_CPU','period_clock':'HOST_WALL' if task in PERIODS else None,
             **ecdf_budget(values)}
        all_work.append(row)
        if task in NAMES: parameters[task]=row
    parameters={k:parameters[k] for k in ORDER}
    fields=list(next(iter(parameters.values())))
    for name,rows in [('extracted-parameters.csv',list(parameters.values())),('extracted-subjob-parameters.csv',all_work)]:
        with (EVIDENCE/name).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');w.writeheader();w.writerows(rows)
    data={'generated_utc':datetime.now(timezone.utc).isoformat(),'source_summary_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
          'source_cohort':summary['main_measurement_cohort'],'successful_laps':len(summary['trials']),
          'quantile_definition':'inf{x: empirical_CDF(x)>=0.95}; ceil(95*N/100)-th order statistic, without interpolation',
          'unit':'s','C_semantics':'user-assumed model WCET; inclusive named-work thread CPU cost',
          'T_semantics':'configured nominal HOST wall period; null for aperiodic work',
          'deadlines':'unassigned','priorities':'unassigned; no inference from CPU execution order',
          'background_policy':'Unlisted/unresolved framework work runs unchanged as pass-through background work.',
          'nested_cost_policy':'Do not charge nested-inclusive children in addition to their inclusive parent.',
          'periodic_named_budget_utilization':sum(parameters[k]['C_model_s']/t for k,t in PERIODS.items()),
          'nominal_periodic_hyperperiod_s':1.,'tasks':parameters}
    write_json(EVIDENCE/'extracted-parameters.json',data)
    model=scheduling_graph(parameters); observed=measured_graph(parameters); cdf_figure(parameters,arrays)
    for svg in FIGURES.glob('*.svg'):
        svg.write_text('\n'.join(line.rstrip() for line in svg.read_text().splitlines())+'\n')
    render_report(data,parameters,model,observed)
    validation={'primary_tasks':len(parameters),'successful_laps':len(summary['trials']),
                'all_CDF_inverse_checks_passed':True,'all_source_job_counts_match':True,
                'periodic_tasks':len(PERIODS),'aperiodic_tasks':len(parameters)-len(PERIODS),
                'model_graph_vertices':len(model['nodes']),'model_graph_edges':len(model['edges']),
                'measured_graph_vertices':len(observed['nodes']),'measured_graph_edges':len(observed['edges']),
                'removed_cpu_order_edges':observed['removed_same_thread_edges'],'both_graphs_acyclic':True,
                'new_simulations':0}
    write_json(EVIDENCE/'extracted-parameters-validation.json',validation)
    print(json.dumps(validation,indent=2))
    for task,p in parameters.items(): print(p['task_id'],task,'C_s',number(p['C_model_s']),'T_s',p['T_nominal_s'],'CDF',p['cdf_at_C'])


if __name__=='__main__':
    main()
