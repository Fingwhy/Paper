import lxml.html, re, json

doc = lxml.html.parse('raw/page.html').getroot()
art = doc.xpath('//article')[0]

def math_latex(el):
    return el.get('alttext') or ''

def inner(el):
    parts = []
    def walk(node):
        if node.tag == 'math':
            parts.append('$' + math_latex(node) + '$')
        else:
            if node.text:
                parts.append(node.text)
            for ch in node:
                walk(ch)
                if ch.tail:
                    parts.append(ch.tail)
    if el.text:
        parts.append(el.text)
    for ch in el:
        walk(ch)
        if ch.tail:
            parts.append(ch.tail)
    return re.sub(r'\s+', ' ', ''.join(parts)).strip()

blocks = []

def handle_list(ul):
    items = [inner(li) for li in ul.xpath('./li')]
    blocks.append({'t': 'list', 'items': items, 'ordered': ul.tag == 'ol'})

def handle_para(div):
    for ch in div:
        cls = ch.get('class') or ''
        if ch.tag == 'p':
            blocks.append({'t': 'p', 'txt': inner(ch)})
        elif ch.tag == 'table' and 'ltx_equation' in cls:
            handle_eq(ch)
        elif ch.tag == 'figure':
            if 'ltx_figure' in cls:
                handle_figure(ch)
            elif 'ltx_table' in cls:
                handle_table(ch)
            else:
                handle_table_or_figure(ch)
        elif ch.tag in ('ul', 'ol'):
            handle_list(ch)
        elif ch.tag == 'div':
            if 'ltx_para' in cls:
                handle_para(ch)
            else:
                walk(ch)

def handle_figure(fig):
    cap = fig.xpath('.//figcaption')
    blocks.append({'t': 'figure', 'imgs': fig.xpath('.//img/@src'),
                   'cap': inner(cap[0]) if cap else ''})

def handle_table(tab):
    cap = tab.xpath('.//figcaption')
    rows = []
    for tr in tab.xpath('.//tr'):
        cells = []
        for td in tr.xpath('./td|./th'):
            try: span = int(td.get('colspan') or 1)
            except: span = 1
            cells.append({'txt': inner(td), 'span': span, 'th': td.tag == 'th'})
        if cells:
            rows.append(cells)
    blocks.append({'t': 'table', 'cap': inner(cap[0]) if cap else '', 'rows': rows})

def handle_table_or_figure(el):
    handle_table(el)

def handle_eq(tab):
    m = tab.xpath('.//math')
    tags = tab.xpath('.//*[contains(@class,"ltx_tag_equation")]')
    blocks.append({'t': 'eq', 'latex': math_latex(m[0]) if m else '',
                   'tag': inner(tags[0]) if tags else ''})

def walk(el):
    for ch in el:
        cls = ch.get('class') or ''
        if ch.tag in ('h1', 'h2', 'h3', 'h4', 'h5', 'h6'):
            blocks.append({'t': 'h', 'lvl': int(ch.tag[1]), 'txt': inner(ch)})
        elif ch.tag == 'section':
            walk(ch)
        elif ch.tag == 'div' and 'ltx_para' in cls:
            handle_para(ch)
        elif ch.tag == 'div':
            walk(ch)
        elif ch.tag in ('ul', 'ol'):
            handle_list(ch)

# 作者信息
title = doc.xpath('//h1[@class="ltx_title ltx_title_document"]')
blocks.append({'t': 'title', 'txt': inner(title[0]) if title else ''})
auth = doc.xpath('//div[contains(@class,"ltx_authors")]')
for a in auth:
    for p in a.xpath('.//span[contains(@class,"ltx_personname")]'):
        blocks.append({'t': 'author', 'txt': inner(p)})
    for p in a.xpath('.//span[contains(@class,"ltx_contact")]|.//span[contains(@class,"ltx_role_contact")]'):
        t = inner(p)
        if t:
            blocks.append({'t': 'affil', 'txt': t})

absdiv = doc.xpath('//div[contains(@class,"ltx_abstract")]')
for a in absdiv:
    for p in a.xpath('./p'):
        blocks.append({'t': 'abs', 'txt': inner(p)})

walk(art)

# 参考文献
for li in doc.xpath('//ul[contains(@class,"ltx_biblist")]/li'):
    blocks.append({'t': 'ref', 'txt': inner(li)})

json.dump(blocks, open('raw/blocks2.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
with open('raw/blocks2.txt', 'w', encoding='utf-8') as f:
    for b in blocks:
        t = b['t']
        if t == 'h':
            f.write('\n' + '#'*b['lvl'] + ' ' + b['txt'] + '\n')
        elif t in ('p', 'abs'):
            f.write('[' + t + '] ' + b['txt'] + '\n')
        elif t == 'list':
            f.write('[LIST]\n')
            for it in b['items']:
                f.write('   - ' + it + '\n')
        elif t == 'figure':
            f.write('[FIG] ' + str(b['imgs']) + ' cap=' + b['cap'] + '\n')
        elif t == 'eq':
            f.write('[EQ %s] %s\n' % (b['tag'], b['latex']))
        elif t == 'table':
            f.write('[TABLE] cap=' + b['cap'] + '\n')
            for r in b['rows']:
                f.write('   | ' + ' | '.join(c['txt'] for c in r) + '\n')
        elif t == 'title':
            f.write('[TITLE] ' + b['txt'] + '\n')
        elif t == 'author':
            f.write('[AUTHOR] ' + b['txt'] + '\n')
        elif t == 'affil':
            f.write('[AFFIL] ' + b['txt'] + '\n')
        elif t == 'ref':
            f.write('[REF] ' + b['txt'] + '\n')
print('blocks:', len(blocks))
