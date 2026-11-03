// Run through the connected CUA browser runtime, passing the report Tab handle.
import assert from 'node:assert/strict';

export async function verifyNumericInputs(tab) {
  const checked=[];
  await tab.playwright.locator('#vertex-index').fill('79224',{timeoutMs:3000});
  for(const [input,index] of [['1.5',1],['-5',0],['100000',79224],['',0],['12.9',12]]){
    await tab.playwright.locator('#vertex-index').fill(input,{timeoutMs:3000});
    const text=await tab.playwright.locator('#vertex-info').innerText();
    assert.match(text,new RegExp(`顶点 ${index}/79224`),`顶点输入 ${JSON.stringify(input)} 应归一到整数 ${index}`);
    checked.push({control:'vertex-index',input,index});
  }
  await tab.playwright.locator('#cell-select').selectOption('38',{timeoutMs:3000});
  for(const [input,index] of [['1.5',1],['-5',0],['100000',3067],['',0],['12.9',12]]){
    await tab.playwright.locator('#raw-index').fill(input,{timeoutMs:3000});
    const text=await tab.playwright.locator('#raw-info').innerText();
    assert.match(text,new RegExp(`索引 ${index}/3067`),`原生目标输入 ${JSON.stringify(input)} 应归一到整数 ${index}`);
    checked.push({control:'raw-index',input,index});
  }
  return checked;
}

export async function verifyAllSelections(tab) {
  const result={cells:0,events:0,orders:0,filters:{}};
  for(let i=0;i<80;i++){
    await tab.playwright.locator('#cell-select').selectOption(String(i),{timeoutMs:3000});
    const name='C'+String(i).padStart(2,'0');
    assert.ok((await tab.playwright.locator('#cell-info').innerText()).startsWith(name+' '));
    result.cells++;
  }
  for(let i=0;i<66;i++){
    await tab.playwright.locator('#event-select').selectOption(String(i),{timeoutMs:3000});
    assert.ok((await tab.playwright.locator('#event-info').innerText()).startsWith('E'+String(i+1).padStart(2,'0')+' /'));
    result.events++;
  }
  for(let i=0;i<61;i++){
    await tab.playwright.locator('#order-select').selectOption(String(i),{timeoutMs:3000});
    assert.ok((await tab.playwright.locator('#order-info').innerText()).startsWith('第 '+(i+1)+'/61 轮'));
    assert.equal(await tab.playwright.locator('#candidate-table .winning').count(),1);
    result.orders++;
  }
  for(const [filter,expected] of [['visited',61],['missing',19],['all',80]]){
    await tab.playwright.locator('#cell-filter').selectOption(filter,{timeoutMs:3000});
    result.filters[filter]=await tab.playwright.locator('#cells-table tr').count();
    assert.equal(result.filters[filter],expected);
  }
  return result;
}

export async function verifyPlaybackControls(tab) {
  const checked=[];
  for(const [range,info,first,last] of [
    ['#event-range','#event-info','E01 /','E66 /'],
    ['#order-range','#order-info','第 1/61','第 61/61'],
    ['#vertex-range','#vertex-info','顶点 0/79224','顶点 79224/79224']]){
    for(const [key,prefix] of [['Home',first],['End',last]]){
      await tab.playwright.locator(range).press(key,{timeoutMs:3000});
      assert.ok((await tab.playwright.locator(info).innerText()).startsWith(prefix));
      checked.push({control:range,key,expected:prefix});
    }
  }
  await tab.playwright.locator('#cell-select').selectOption('38');
  for(const [key,prefix] of [['Home','索引 0/3067'],['End','索引 3067/3067']]){
    await tab.playwright.locator('#raw-progress').press(key,{timeoutMs:3000});
    assert.ok((await tab.playwright.locator('#raw-info').innerText()).startsWith(prefix));
    checked.push({control:'#raw-progress',key,expected:prefix});
  }
  for(const [cell,button] of [['0','#cell-prev'],['79','#cell-next']]){
    await tab.playwright.locator('#cell-select').selectOption(cell);
    await tab.playwright.locator(button).click({timeoutMs:3000});
    assert.ok((await tab.playwright.locator('#cell-info').innerText()).startsWith('C'+cell.padStart(2,'0')+' '));
    checked.push({control:button,expectedCell:Number(cell)});
  }
  for(const id of ['show-cells','show-f','show-grid','show-raw','show-safe','show-final','show-labels']){
    const checkbox=tab.playwright.locator('#'+id);
    const before=await checkbox.evaluate(el=>el.checked);
    await checkbox.setChecked(!before,{timeoutMs:3000});
    assert.equal(await checkbox.evaluate(el=>el.checked),!before);
    await checkbox.setChecked(before,{timeoutMs:3000});
    assert.equal(await checkbox.evaluate(el=>el.checked),before);
    checked.push({control:id,toggleAndRestore:true});
  }
  return checked;
}
