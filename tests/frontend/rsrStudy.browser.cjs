// Run against a static SPA server with the production frontend build.
// API replies are mocked; microphone recording, IndexedDB, and windows are real.
const { chromium } = require(process.env.BLOOM_PLAYWRIGHT_PATH || 'playwright');
const assert = require('node:assert/strict');
(async () => {
  const browser = await chromium.launch({ headless: true, args: ['--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'] });
  try {
    const context = await browser.newContext({ permissions: ['microphone'], viewport: {width:1440,height:1100} });
    let loggedIn = false;
    const researcher = {id:'browser-examiner',role:'Researcher',userName:'issued-researcher',fullName:'Study Examiner'};
    const manifest = Array.from({length:16}, (_,i) => ({id:i+1,text:`Study sentence ${i+1}.`}));
    let failAnalysis = true;
    await context.route('**/api/**', async route => {
      const url = route.request().url();
      if (url.endsWith('/rsr-study/auth/login')) {
        const credentials = route.request().postDataJSON();
        if (credentials.login !== 'issued-researcher' || credentials.password !== 'Assigned@456')
          return route.fulfill({status:401,json:{message:'Invalid researcher login or password.'}});
        loggedIn = true; return route.fulfill({json:{user:researcher}});
      }
      if (url.endsWith('/rsr-study/auth/session')) return loggedIn
        ? route.fulfill({json:researcher}) : route.fulfill({status:401,body:'Unauthorized'});
      if (url.endsWith('/rsr-study/auth/logout')) { loggedIn = false; return route.fulfill({status:204}); }
      if (url.endsWith('/rsr-speech/sentences')) return route.fulfill({json:manifest});
      if (url.endsWith('/rsr/analyze')) {
        if (failAnalysis) return route.fulfill({status:502,body:'Service offline'});
        return route.fulfill({json:{assessmentId:'assessment-1',totalScore:2,decision:'N/A',sentences:[{id:'1',score:2,response:'Study sentence one.'}]}});
      }
      return route.fulfill({json:[]});
    });
    const page = await context.newPage();
    const errors = []; page.on('pageerror', e => errors.push(e.message));
    await page.goto(`${process.env.BLOOM_URL || 'http://127.0.0.1:3097'}/rsr-study`);
    await page.getByRole('heading',{name:'Researcher sign in'}).waitFor();
    await page.getByLabel('Researcher login',{exact:true}).fill('issued-researcher');
    await page.getByLabel('Password',{exact:true}).fill('Wrong@123');
    await page.getByRole('button',{name:'Sign in to study'}).click();
    await page.getByRole('alert').filter({hasText:'incorrect'}).waitFor();
    await page.getByLabel('Password',{exact:true}).fill('Assigned@456');
    await page.getByRole('button',{name:'Sign in to study'}).click();
    await page.getByLabel('Participant code').fill('P-BROWSER');
    await page.getByLabel('Age in months').fill('72');
    await page.getByRole('button',{name:'Create session',exact:true}).click();
    assert(await page.getByRole('button',{name:'Cue examiner prompt'}).isDisabled());
    const popupPromise = page.waitForEvent('popup');
    await page.getByRole('button',{name:'Open child display'}).click();
    const child = await popupPromise;
    await child.getByRole('heading',{name:'Let’s get ready'}).waitFor();
    assert.equal(await child.locator('header').count(), 0);
    assert.equal(await child.getByText('Study sentence 1.').count(), 0);
    await page.getByRole('button',{name:'Start continuous recording'}).click();
    await page.getByText('● Continuous recording is running',{exact:false}).waitFor();
    await page.waitForFunction(async () => {
      const db = await new Promise(resolve => {const r=indexedDB.open('bloom-rsr-study',1);r.onsuccess=()=>resolve(r.result);});
      const count = await new Promise(resolve=>{const r=db.transaction('chunks').objectStore('chunks').count();r.onsuccess=()=>resolve(r.result);});
      db.close(); return count>0;
    });
    await page.getByRole('button',{name:'Cue examiner prompt'}).click();
    await child.getByRole('heading',{name:'Listen carefully'}).waitFor();
    await page.getByRole('button',{name:'Prompt ended / start response window'}).click();
    await child.getByRole('heading',{name:'Your turn'}).waitFor();
    await page.waitForTimeout(200);
    await page.getByRole('button',{name:'End attempt',exact:true}).click();
    await page.getByRole('button',{name:'Break (keep recording)',exact:true}).click();
    await child.getByRole('heading',{name:'We’re taking a break'}).waitFor();
    await page.getByRole('button',{name:'Resume',exact:true}).click();
    await page.getByRole('button',{name:'Finish recording',exact:true}).click();
    await child.getByRole('heading',{name:'Thank you!'}).waitFor();
    await page.locator('audio').waitFor();
    await page.waitForFunction(() => document.querySelector('audio')?.readyState >= 1);
    assert(await page.locator('audio').evaluate(a => a.duration > 0));
    await page.getByRole('button',{name:'Analyze response windows'}).click();
    await page.getByRole('alert').filter({hasText:'502'}).waitFor();
    const src = await page.locator('audio').getAttribute('src');
    assert(src.startsWith('blob:'));
    failAnalysis = false;
    await page.getByRole('button',{name:'Analyze response windows'}).click();
    await page.getByRole('heading',{name:/Automatic score:/}).waitFor();
    await page.getByLabel('Reason for override or exclusion').fill('Examiner reviewed the full recording');
    await page.getByLabel('Reviewed score').selectOption('0');
    await page.getByRole('button',{name:'Apply score override'}).click();
    assert((await page.locator('tbody tr').first().innerText()).includes('Examiner reviewed the full recording'));
    await page.screenshot({path:process.env.BLOOM_SCREENSHOT || '.test-tools/study-admin.png',fullPage:true});
    await child.screenshot({path:'.test-tools/study-child.png',fullPage:true});
    await page.reload();
    await page.getByRole('button',{name:'Review / recover'}).click();
    await page.locator('audio').waitFor();
    assert((await page.locator('tbody tr').first().innerText()).includes('Examiner reviewed the full recording'));
    assert.equal(errors.length,0, errors.join('\n'));
    await page.getByRole('button',{name:'Sign out of study'}).click();
    await page.getByRole('heading',{name:'Researcher sign in'}).waitFor();
    assert.equal(loggedIn,false);
    console.log('PASS: issued-login gate, bad-password rejection, early audio, continuous capture, child sync, playable final audio, scoring retry, override, reload recovery, and sign out.');
    await context.close();
  } finally { await browser.close(); }
})().catch(e => {console.error(e);process.exitCode=1;});
