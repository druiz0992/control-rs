use crate::utils::matrix::vec_to_dmat;
use nalgebra::DMatrix;

pub fn eval_dfdx(params: &[f64]) -> DMatrix<f64> {
    let mut v: Vec<Vec<f64>> = vec![vec![0.0; 4]; 4];
    let t0 = 0.00000000000000000e0;
    v[0][0] = t0;
    let t1 = 1.00000000000000000e0;
    v[0][1] = t1;
    let t2 = 0.00000000000000000e0;
    v[0][2] = t2;
    let t3 = 0.00000000000000000e0;
    v[0][3] = t3;
    let t4 = params[10];
    let t5 = params[1];
    let t6 = params[1];
    let t7 = t6.powf(2.00000000000000000e0);
    let t8 = t7 + 1.00000000000000004e-10;
    let t9 = t8.powf(5.00000000000000000e-1);
    let t10 = -t4;
    let t11 = 1.0 / t9;
    let t12 = t10 * t11;
    let t13 = t5.powf(3.00000000000000000e0);
    let t14 = params[1];
    let t15 = params[2];
    let t16 = params[0];
    let t17 = -t15;
    let t18 = t16 + t17;
    let t19 = params[8];
    let t20 = t14.powf(2.00000000000000000e0);
    let t21 = t19 * t20;
    let t22 = t18.cos();
    let t23 = params[3];
    let t24 = params[9];
    let t25 = t23.powf(2.00000000000000000e0);
    let t26 = t21 * t22;
    let t27 = t24 * t25;
    let t28 = params[2];
    let t29 = params[0];
    let t30 = -t28;
    let t31 = t29 + t30;
    let t32 = params[7];
    let t33 = t26 + t27;
    let t34 = t32 * t33;
    let t35 = t31.sin();
    let t36 = t34 * t35;
    let t37 = params[2];
    let t38 = params[2];
    let t39 = params[0];
    let t40 = -t38;
    let t41 = t39 + t40;
    let t42 = params[7];
    let t43 = t37.sin();
    let t44 = t42 * t43;
    let t45 = t41.cos();
    let t46 = t44 * t45;
    let t47 = params[6];
    let t48 = params[7];
    let t49 = t47 * 9.81000000000000050e0;
    let t50 = t48 * 9.81000000000000050e0;
    let t51 = params[0];
    let t52 = t49 + t50;
    let t53 = t51.sin();
    let t54 = t52 * t53;
    let t55 = -t36;
    let t56 = t12 * t13;
    let t57 = t55 + t56;
    let t58 = t46 * 9.81000000000000050e0;
    let t59 = t57 + t58;
    let t60 = -t54;
    let t61 = t59 + t60;
    let t62 = params[4];
    let t63 = params[2];
    let t64 = params[0];
    let t65 = -t63;
    let t66 = t64 + t65;
    let t67 = params[2];
    let t68 = params[0];
    let t69 = -t67;
    let t70 = t68 + t69;
    let t71 = params[2];
    let t72 = params[0];
    let t73 = -t71;
    let t74 = t72 + t73;
    let t75 = t74.sin();
    let t76 = params[7];
    let t77 = t75.powf(2.00000000000000000e0);
    let t78 = params[6];
    let t79 = t76 * t77;
    let t80 = t78 + t79;
    let t81 = params[8];
    let t82 = t80.powf(2.00000000000000000e0);
    let t83 = t81 * t82;
    let t84 = params[7];
    let t85 = t61 + t62;
    let t86 = t84 * t85;
    let t87 = t66.sin();
    let t88 = t86 * t87;
    let t89 = 1.0 / t83;
    let t90 = t88 * t89;
    let t91 = t70.cos();
    let t92 = t90 * t91;
    let t93 = params[1];
    let t94 = params[2];
    let t95 = params[0];
    let t96 = -t94;
    let t97 = t95 + t96;
    let t98 = t97.sin();
    let t99 = params[8];
    let t100 = params[7];
    let t101 = t99 * t100;
    let t102 = t93.powf(2.00000000000000000e0);
    let t103 = t101 * t102;
    let t104 = t98.powf(2.00000000000000000e0);
    let t105 = params[1];
    let t106 = params[2];
    let t107 = params[0];
    let t108 = -t106;
    let t109 = t107 + t108;
    let t110 = params[8];
    let t111 = t105.powf(2.00000000000000000e0);
    let t112 = t110 * t111;
    let t113 = t109.cos();
    let t114 = params[3];
    let t115 = params[9];
    let t116 = t114.powf(2.00000000000000000e0);
    let t117 = t112 * t113;
    let t118 = t115 * t116;
    let t119 = params[2];
    let t120 = params[0];
    let t121 = -t119;
    let t122 = t120 + t121;
    let t123 = params[7];
    let t124 = t117 + t118;
    let t125 = t123 * t124;
    let t126 = t122.cos();
    let t127 = t125 * t126;
    let t128 = params[2];
    let t129 = params[2];
    let t130 = params[0];
    let t131 = -t129;
    let t132 = t130 + t131;
    let t133 = params[7];
    let t134 = t128.sin();
    let t135 = t133 * t134;
    let t136 = t132.sin();
    let t137 = t135 * t136;
    let t138 = t137 * 9.81000000000000050e0;
    let t139 = params[6];
    let t140 = params[7];
    let t141 = t139 * 9.81000000000000050e0;
    let t142 = t140 * 9.81000000000000050e0;
    let t143 = params[0];
    let t144 = t141 + t142;
    let t145 = t143.cos();
    let t146 = t144 * t145;
    let t147 = t103 * t104;
    let t148 = -t127;
    let t149 = t147 + t148;
    let t150 = -t138;
    let t151 = t149 + t150;
    let t152 = -t146;
    let t153 = params[2];
    let t154 = params[0];
    let t155 = -t153;
    let t156 = t154 + t155;
    let t157 = t156.sin();
    let t158 = params[7];
    let t159 = t157.powf(2.00000000000000000e0);
    let t160 = params[6];
    let t161 = t158 * t159;
    let t162 = params[8];
    let t163 = t160 + t161;
    let t164 = t162 * t163;
    let t165 = t151 + t152;
    let t166 = 1.0 / t164;
    let t167 = t92 * -2.00000000000000000e0;
    let t168 = t165 * t166;
    let t169 = t167 + t168;
    v[1][0] = t169;
    let t170 = params[1];
    let t171 = params[1];
    let t172 = t171.powf(2.00000000000000000e0);
    let t173 = t172 + 1.00000000000000004e-10;
    let t174 = t173.powf(1.50000000000000000e0);
    let t175 = params[10];
    let t176 = 1.0 / t174;
    let t177 = t175 * t176;
    let t178 = t170.powf(4.00000000000000000e0);
    let t179 = params[1];
    let t180 = params[1];
    let t181 = t180.powf(2.00000000000000000e0);
    let t182 = t181 + 1.00000000000000004e-10;
    let t183 = t182.powf(5.00000000000000000e-1);
    let t184 = params[10];
    let t185 = 1.0 / t183;
    let t186 = t184 * t185;
    let t187 = t179.powf(2.00000000000000000e0);
    let t188 = t186 * t187;
    let t189 = t188 * 3.00000000000000000e0;
    let t190 = params[2];
    let t191 = params[0];
    let t192 = -t190;
    let t193 = t191 + t192;
    let t194 = params[2];
    let t195 = params[0];
    let t196 = -t194;
    let t197 = t195 + t196;
    let t198 = params[8];
    let t199 = params[7];
    let t200 = t198 * t199;
    let t201 = params[1];
    let t202 = t200 * t201;
    let t203 = t193.sin();
    let t204 = t202 * t203;
    let t205 = t197.cos();
    let t206 = t204 * t205;
    let t207 = t206 * 2.00000000000000000e0;
    let t208 = t177 * t178;
    let t209 = -t189;
    let t210 = t208 + t209;
    let t211 = -t207;
    let t212 = params[2];
    let t213 = params[0];
    let t214 = -t212;
    let t215 = t213 + t214;
    let t216 = t215.sin();
    let t217 = params[7];
    let t218 = t216.powf(2.00000000000000000e0);
    let t219 = params[6];
    let t220 = t217 * t218;
    let t221 = params[8];
    let t222 = t219 + t220;
    let t223 = t221 * t222;
    let t224 = t210 + t211;
    let t225 = 1.0 / t223;
    let t226 = t224 * t225;
    v[1][1] = t226;
    let t227 = params[10];
    let t228 = params[1];
    let t229 = params[1];
    let t230 = t229.powf(2.00000000000000000e0);
    let t231 = t230 + 1.00000000000000004e-10;
    let t232 = t231.powf(5.00000000000000000e-1);
    let t233 = -t227;
    let t234 = 1.0 / t232;
    let t235 = t233 * t234;
    let t236 = t228.powf(3.00000000000000000e0);
    let t237 = params[1];
    let t238 = params[2];
    let t239 = params[0];
    let t240 = -t238;
    let t241 = t239 + t240;
    let t242 = params[8];
    let t243 = t237.powf(2.00000000000000000e0);
    let t244 = t242 * t243;
    let t245 = t241.cos();
    let t246 = params[3];
    let t247 = params[9];
    let t248 = t246.powf(2.00000000000000000e0);
    let t249 = t244 * t245;
    let t250 = t247 * t248;
    let t251 = params[2];
    let t252 = params[0];
    let t253 = -t251;
    let t254 = t252 + t253;
    let t255 = params[7];
    let t256 = t249 + t250;
    let t257 = t255 * t256;
    let t258 = t254.sin();
    let t259 = t257 * t258;
    let t260 = params[2];
    let t261 = params[2];
    let t262 = params[0];
    let t263 = -t261;
    let t264 = t262 + t263;
    let t265 = params[7];
    let t266 = t260.sin();
    let t267 = t265 * t266;
    let t268 = t264.cos();
    let t269 = t267 * t268;
    let t270 = params[6];
    let t271 = params[7];
    let t272 = t270 * 9.81000000000000050e0;
    let t273 = t271 * 9.81000000000000050e0;
    let t274 = params[0];
    let t275 = t272 + t273;
    let t276 = t274.sin();
    let t277 = t275 * t276;
    let t278 = -t259;
    let t279 = t235 * t236;
    let t280 = t278 + t279;
    let t281 = t269 * 9.81000000000000050e0;
    let t282 = t280 + t281;
    let t283 = -t277;
    let t284 = t282 + t283;
    let t285 = params[4];
    let t286 = params[2];
    let t287 = params[0];
    let t288 = -t286;
    let t289 = t287 + t288;
    let t290 = params[2];
    let t291 = params[0];
    let t292 = -t290;
    let t293 = t291 + t292;
    let t294 = params[2];
    let t295 = params[0];
    let t296 = -t294;
    let t297 = t295 + t296;
    let t298 = t297.sin();
    let t299 = params[7];
    let t300 = t298.powf(2.00000000000000000e0);
    let t301 = params[6];
    let t302 = t299 * t300;
    let t303 = t301 + t302;
    let t304 = params[8];
    let t305 = t303.powf(2.00000000000000000e0);
    let t306 = t304 * t305;
    let t307 = params[7];
    let t308 = t284 + t285;
    let t309 = t307 * t308;
    let t310 = t289.sin();
    let t311 = t309 * t310;
    let t312 = 1.0 / t306;
    let t313 = t311 * t312;
    let t314 = t293.cos();
    let t315 = t313 * t314;
    let t316 = params[8];
    let t317 = params[1];
    let t318 = params[2];
    let t319 = params[0];
    let t320 = -t318;
    let t321 = t319 + t320;
    let t322 = t321.sin();
    let t323 = -t316;
    let t324 = params[7];
    let t325 = t323 * t324;
    let t326 = t317.powf(2.00000000000000000e0);
    let t327 = t325 * t326;
    let t328 = t322.powf(2.00000000000000000e0);
    let t329 = params[1];
    let t330 = params[2];
    let t331 = params[0];
    let t332 = -t330;
    let t333 = t331 + t332;
    let t334 = params[8];
    let t335 = t329.powf(2.00000000000000000e0);
    let t336 = t334 * t335;
    let t337 = t333.cos();
    let t338 = params[3];
    let t339 = params[9];
    let t340 = t338.powf(2.00000000000000000e0);
    let t341 = t336 * t337;
    let t342 = t339 * t340;
    let t343 = params[2];
    let t344 = params[0];
    let t345 = -t343;
    let t346 = t344 + t345;
    let t347 = params[7];
    let t348 = t341 + t342;
    let t349 = t347 * t348;
    let t350 = t346.cos();
    let t351 = params[2];
    let t352 = params[2];
    let t353 = params[0];
    let t354 = -t352;
    let t355 = t353 + t354;
    let t356 = params[7];
    let t357 = t351.sin();
    let t358 = t356 * t357;
    let t359 = t355.sin();
    let t360 = t358 * t359;
    let t361 = params[2];
    let t362 = params[2];
    let t363 = params[0];
    let t364 = -t362;
    let t365 = t363 + t364;
    let t366 = params[7];
    let t367 = t361.cos();
    let t368 = t366 * t367;
    let t369 = t365.cos();
    let t370 = t368 * t369;
    let t371 = t327 * t328;
    let t372 = t349 * t350;
    let t373 = t371 + t372;
    let t374 = t360 * 9.81000000000000050e0;
    let t375 = t373 + t374;
    let t376 = t370 * 9.81000000000000050e0;
    let t377 = params[2];
    let t378 = params[0];
    let t379 = -t377;
    let t380 = t378 + t379;
    let t381 = t380.sin();
    let t382 = params[7];
    let t383 = t381.powf(2.00000000000000000e0);
    let t384 = params[6];
    let t385 = t382 * t383;
    let t386 = params[8];
    let t387 = t384 + t385;
    let t388 = t386 * t387;
    let t389 = t375 + t376;
    let t390 = 1.0 / t388;
    let t391 = t315 * 2.00000000000000000e0;
    let t392 = t389 * t390;
    let t393 = t391 + t392;
    v[1][2] = t393;
    let t394 = params[2];
    let t395 = params[0];
    let t396 = -t394;
    let t397 = t395 + t396;
    let t398 = params[2];
    let t399 = params[0];
    let t400 = -t398;
    let t401 = t399 + t400;
    let t402 = t401.sin();
    let t403 = params[7];
    let t404 = t402.powf(2.00000000000000000e0);
    let t405 = params[6];
    let t406 = t403 * t404;
    let t407 = params[8];
    let t408 = t405 + t406;
    let t409 = t407 * t408;
    let t410 = params[9];
    let t411 = params[7];
    let t412 = t410 * t411;
    let t413 = params[3];
    let t414 = t412 * t413;
    let t415 = 1.0 / t409;
    let t416 = t414 * t415;
    let t417 = t397.sin();
    let t418 = t416 * t417;
    let t419 = t418 * -2.00000000000000000e0;
    v[1][3] = t419;
    let t420 = 0.00000000000000000e0;
    v[2][0] = t420;
    let t421 = 0.00000000000000000e0;
    v[2][1] = t421;
    let t422 = 0.00000000000000000e0;
    v[2][2] = t422;
    let t423 = 1.00000000000000000e0;
    v[2][3] = t423;
    let t424 = params[10];
    let t425 = params[3];
    let t426 = params[3];
    let t427 = t426.powf(2.00000000000000000e0);
    let t428 = t427 + 1.00000000000000004e-10;
    let t429 = t428.powf(5.00000000000000000e-1);
    let t430 = -t424;
    let t431 = 1.0 / t429;
    let t432 = t430 * t431;
    let t433 = t425.powf(3.00000000000000000e0);
    let t434 = params[3];
    let t435 = params[2];
    let t436 = params[0];
    let t437 = -t435;
    let t438 = t436 + t437;
    let t439 = params[2];
    let t440 = params[0];
    let t441 = -t439;
    let t442 = t440 + t441;
    let t443 = params[9];
    let t444 = params[7];
    let t445 = t443 * t444;
    let t446 = t434.powf(2.00000000000000000e0);
    let t447 = t445 * t446;
    let t448 = t438.sin();
    let t449 = t447 * t448;
    let t450 = t442.cos();
    let t451 = params[6];
    let t452 = params[7];
    let t453 = params[1];
    let t454 = params[2];
    let t455 = params[0];
    let t456 = -t454;
    let t457 = t455 + t456;
    let t458 = params[8];
    let t459 = t453.powf(2.00000000000000000e0);
    let t460 = t458 * t459;
    let t461 = t457.sin();
    let t462 = params[0];
    let t463 = params[2];
    let t464 = params[0];
    let t465 = -t463;
    let t466 = t464 + t465;
    let t467 = t462.sin();
    let t468 = t466.cos();
    let t469 = t467 * t468;
    let t470 = params[2];
    let t471 = t470.sin();
    let t472 = t471 * 9.81000000000000050e0;
    let t473 = t460 * t461;
    let t474 = -t472;
    let t475 = t473 + t474;
    let t476 = t469 * 9.81000000000000050e0;
    let t477 = t451 + t452;
    let t478 = t475 + t476;
    let t479 = t432 * t433;
    let t480 = t449 * t450;
    let t481 = t479 + t480;
    let t482 = params[5];
    let t483 = t481 + t482;
    let t484 = t477 * t478;
    let t485 = params[2];
    let t486 = params[0];
    let t487 = -t485;
    let t488 = t486 + t487;
    let t489 = params[2];
    let t490 = params[0];
    let t491 = -t489;
    let t492 = t490 + t491;
    let t493 = params[2];
    let t494 = params[0];
    let t495 = -t493;
    let t496 = t494 + t495;
    let t497 = t496.sin();
    let t498 = params[7];
    let t499 = t497.powf(2.00000000000000000e0);
    let t500 = params[6];
    let t501 = t498 * t499;
    let t502 = t500 + t501;
    let t503 = params[9];
    let t504 = t502.powf(2.00000000000000000e0);
    let t505 = t503 * t504;
    let t506 = params[7];
    let t507 = t483 + t484;
    let t508 = t506 * t507;
    let t509 = t488.sin();
    let t510 = t508 * t509;
    let t511 = 1.0 / t505;
    let t512 = t510 * t511;
    let t513 = t492.cos();
    let t514 = t512 * t513;
    let t515 = params[9];
    let t516 = params[3];
    let t517 = params[2];
    let t518 = params[0];
    let t519 = -t517;
    let t520 = t518 + t519;
    let t521 = t520.sin();
    let t522 = -t515;
    let t523 = params[7];
    let t524 = t522 * t523;
    let t525 = t516.powf(2.00000000000000000e0);
    let t526 = t524 * t525;
    let t527 = t521.powf(2.00000000000000000e0);
    let t528 = params[3];
    let t529 = params[2];
    let t530 = params[0];
    let t531 = -t529;
    let t532 = t530 + t531;
    let t533 = t532.cos();
    let t534 = params[9];
    let t535 = params[7];
    let t536 = t534 * t535;
    let t537 = t528.powf(2.00000000000000000e0);
    let t538 = t536 * t537;
    let t539 = t533.powf(2.00000000000000000e0);
    let t540 = params[6];
    let t541 = params[7];
    let t542 = params[1];
    let t543 = params[2];
    let t544 = params[0];
    let t545 = -t543;
    let t546 = t544 + t545;
    let t547 = params[8];
    let t548 = t542.powf(2.00000000000000000e0);
    let t549 = t547 * t548;
    let t550 = t546.cos();
    let t551 = params[0];
    let t552 = params[2];
    let t553 = params[0];
    let t554 = -t552;
    let t555 = t553 + t554;
    let t556 = t551.sin();
    let t557 = t555.sin();
    let t558 = t556 * t557;
    let t559 = t558 * 9.81000000000000050e0;
    let t560 = params[0];
    let t561 = params[2];
    let t562 = params[0];
    let t563 = -t561;
    let t564 = t562 + t563;
    let t565 = t560.cos();
    let t566 = t564.cos();
    let t567 = t565 * t566;
    let t568 = -t559;
    let t569 = t549 * t550;
    let t570 = t568 + t569;
    let t571 = t567 * 9.81000000000000050e0;
    let t572 = t540 + t541;
    let t573 = t570 + t571;
    let t574 = t526 * t527;
    let t575 = t538 * t539;
    let t576 = t574 + t575;
    let t577 = t572 * t573;
    let t578 = params[2];
    let t579 = params[0];
    let t580 = -t578;
    let t581 = t579 + t580;
    let t582 = t581.sin();
    let t583 = params[7];
    let t584 = t582.powf(2.00000000000000000e0);
    let t585 = params[6];
    let t586 = t583 * t584;
    let t587 = params[9];
    let t588 = t585 + t586;
    let t589 = t587 * t588;
    let t590 = t576 + t577;
    let t591 = 1.0 / t589;
    let t592 = t514 * -2.00000000000000000e0;
    let t593 = t590 * t591;
    let t594 = t592 + t593;
    v[3][0] = t594;
    let t595 = params[6];
    let t596 = params[7];
    let t597 = params[2];
    let t598 = params[0];
    let t599 = -t597;
    let t600 = t598 + t599;
    let t601 = params[2];
    let t602 = params[0];
    let t603 = -t601;
    let t604 = t602 + t603;
    let t605 = t604.sin();
    let t606 = params[7];
    let t607 = t605.powf(2.00000000000000000e0);
    let t608 = params[6];
    let t609 = t606 * t607;
    let t610 = params[9];
    let t611 = t608 + t609;
    let t612 = t610 * t611;
    let t613 = params[8];
    let t614 = params[1];
    let t615 = t613 * t614;
    let t616 = t595 + t596;
    let t617 = t615 * t616;
    let t618 = 1.0 / t612;
    let t619 = t617 * t618;
    let t620 = t600.sin();
    let t621 = t619 * t620;
    let t622 = t621 * 2.00000000000000000e0;
    v[3][1] = t622;
    let t623 = params[10];
    let t624 = params[3];
    let t625 = params[3];
    let t626 = t625.powf(2.00000000000000000e0);
    let t627 = t626 + 1.00000000000000004e-10;
    let t628 = t627.powf(5.00000000000000000e-1);
    let t629 = -t623;
    let t630 = 1.0 / t628;
    let t631 = t629 * t630;
    let t632 = t624.powf(3.00000000000000000e0);
    let t633 = params[3];
    let t634 = params[2];
    let t635 = params[0];
    let t636 = -t634;
    let t637 = t635 + t636;
    let t638 = params[2];
    let t639 = params[0];
    let t640 = -t638;
    let t641 = t639 + t640;
    let t642 = params[9];
    let t643 = params[7];
    let t644 = t642 * t643;
    let t645 = t633.powf(2.00000000000000000e0);
    let t646 = t644 * t645;
    let t647 = t637.sin();
    let t648 = t646 * t647;
    let t649 = t641.cos();
    let t650 = params[6];
    let t651 = params[7];
    let t652 = params[1];
    let t653 = params[2];
    let t654 = params[0];
    let t655 = -t653;
    let t656 = t654 + t655;
    let t657 = params[8];
    let t658 = t652.powf(2.00000000000000000e0);
    let t659 = t657 * t658;
    let t660 = t656.sin();
    let t661 = params[0];
    let t662 = params[2];
    let t663 = params[0];
    let t664 = -t662;
    let t665 = t663 + t664;
    let t666 = t661.sin();
    let t667 = t665.cos();
    let t668 = t666 * t667;
    let t669 = params[2];
    let t670 = t669.sin();
    let t671 = t670 * 9.81000000000000050e0;
    let t672 = t659 * t660;
    let t673 = -t671;
    let t674 = t672 + t673;
    let t675 = t668 * 9.81000000000000050e0;
    let t676 = t650 + t651;
    let t677 = t674 + t675;
    let t678 = t631 * t632;
    let t679 = t648 * t649;
    let t680 = t678 + t679;
    let t681 = params[5];
    let t682 = t680 + t681;
    let t683 = t676 * t677;
    let t684 = params[2];
    let t685 = params[0];
    let t686 = -t684;
    let t687 = t685 + t686;
    let t688 = params[2];
    let t689 = params[0];
    let t690 = -t688;
    let t691 = t689 + t690;
    let t692 = params[2];
    let t693 = params[0];
    let t694 = -t692;
    let t695 = t693 + t694;
    let t696 = t695.sin();
    let t697 = params[7];
    let t698 = t696.powf(2.00000000000000000e0);
    let t699 = params[6];
    let t700 = t697 * t698;
    let t701 = t699 + t700;
    let t702 = params[9];
    let t703 = t701.powf(2.00000000000000000e0);
    let t704 = t702 * t703;
    let t705 = params[7];
    let t706 = t682 + t683;
    let t707 = t705 * t706;
    let t708 = t687.sin();
    let t709 = t707 * t708;
    let t710 = 1.0 / t704;
    let t711 = t709 * t710;
    let t712 = t691.cos();
    let t713 = t711 * t712;
    let t714 = params[3];
    let t715 = params[2];
    let t716 = params[0];
    let t717 = -t715;
    let t718 = t716 + t717;
    let t719 = t718.sin();
    let t720 = params[9];
    let t721 = params[7];
    let t722 = t720 * t721;
    let t723 = t714.powf(2.00000000000000000e0);
    let t724 = t722 * t723;
    let t725 = t719.powf(2.00000000000000000e0);
    let t726 = params[3];
    let t727 = params[2];
    let t728 = params[0];
    let t729 = -t727;
    let t730 = t728 + t729;
    let t731 = t730.cos();
    let t732 = params[9];
    let t733 = params[7];
    let t734 = t732 * t733;
    let t735 = t726.powf(2.00000000000000000e0);
    let t736 = t734 * t735;
    let t737 = t731.powf(2.00000000000000000e0);
    let t738 = t736 * t737;
    let t739 = params[6];
    let t740 = params[7];
    let t741 = params[8];
    let t742 = params[1];
    let t743 = params[2];
    let t744 = params[0];
    let t745 = -t743;
    let t746 = t744 + t745;
    let t747 = -t741;
    let t748 = t742.powf(2.00000000000000000e0);
    let t749 = t747 * t748;
    let t750 = t746.cos();
    let t751 = params[0];
    let t752 = params[2];
    let t753 = params[0];
    let t754 = -t752;
    let t755 = t753 + t754;
    let t756 = t751.sin();
    let t757 = t755.sin();
    let t758 = t756 * t757;
    let t759 = params[2];
    let t760 = t759.cos();
    let t761 = t760 * 9.81000000000000050e0;
    let t762 = t749 * t750;
    let t763 = -t761;
    let t764 = t762 + t763;
    let t765 = t758 * 9.81000000000000050e0;
    let t766 = t739 + t740;
    let t767 = t764 + t765;
    let t768 = -t738;
    let t769 = t724 * t725;
    let t770 = t768 + t769;
    let t771 = t766 * t767;
    let t772 = params[2];
    let t773 = params[0];
    let t774 = -t772;
    let t775 = t773 + t774;
    let t776 = t775.sin();
    let t777 = params[7];
    let t778 = t776.powf(2.00000000000000000e0);
    let t779 = params[6];
    let t780 = t777 * t778;
    let t781 = params[9];
    let t782 = t779 + t780;
    let t783 = t781 * t782;
    let t784 = t770 + t771;
    let t785 = 1.0 / t783;
    let t786 = t713 * 2.00000000000000000e0;
    let t787 = t784 * t785;
    let t788 = t786 + t787;
    v[3][2] = t788;
    let t789 = params[3];
    let t790 = params[3];
    let t791 = t790.powf(2.00000000000000000e0);
    let t792 = t791 + 1.00000000000000004e-10;
    let t793 = t792.powf(1.50000000000000000e0);
    let t794 = params[10];
    let t795 = 1.0 / t793;
    let t796 = t794 * t795;
    let t797 = t789.powf(4.00000000000000000e0);
    let t798 = params[3];
    let t799 = params[3];
    let t800 = t799.powf(2.00000000000000000e0);
    let t801 = t800 + 1.00000000000000004e-10;
    let t802 = t801.powf(5.00000000000000000e-1);
    let t803 = params[10];
    let t804 = 1.0 / t802;
    let t805 = t803 * t804;
    let t806 = t798.powf(2.00000000000000000e0);
    let t807 = t805 * t806;
    let t808 = t807 * 3.00000000000000000e0;
    let t809 = params[2];
    let t810 = params[0];
    let t811 = -t809;
    let t812 = t810 + t811;
    let t813 = params[2];
    let t814 = params[0];
    let t815 = -t813;
    let t816 = t814 + t815;
    let t817 = params[9];
    let t818 = params[7];
    let t819 = t817 * t818;
    let t820 = params[3];
    let t821 = t819 * t820;
    let t822 = t812.sin();
    let t823 = t821 * t822;
    let t824 = t816.cos();
    let t825 = t823 * t824;
    let t826 = -t808;
    let t827 = t796 * t797;
    let t828 = t826 + t827;
    let t829 = t825 * 2.00000000000000000e0;
    let t830 = params[2];
    let t831 = params[0];
    let t832 = -t830;
    let t833 = t831 + t832;
    let t834 = t833.sin();
    let t835 = params[7];
    let t836 = t834.powf(2.00000000000000000e0);
    let t837 = params[6];
    let t838 = t835 * t836;
    let t839 = params[9];
    let t840 = t837 + t838;
    let t841 = t839 * t840;
    let t842 = t828 + t829;
    let t843 = 1.0 / t841;
    let t844 = t842 * t843;
    v[3][3] = t844;
    vec_to_dmat(&v)
}

pub fn eval_dfdu(params: &[f64]) -> DMatrix<f64> {
    let mut v: Vec<Vec<f64>> = vec![vec![0.0; 2]; 4];
    let t0 = 0.00000000000000000e0;
    v[0][0] = t0;
    let t1 = 0.00000000000000000e0;
    v[0][1] = t1;
    let t2 = params[2];
    let t3 = params[0];
    let t4 = -t2;
    let t5 = t3 + t4;
    let t6 = t5.sin();
    let t7 = params[7];
    let t8 = t6.powf(2.00000000000000000e0);
    let t9 = params[6];
    let t10 = t7 * t8;
    let t11 = params[8];
    let t12 = t9 + t10;
    let t13 = t11 * t12;
    let t14 = 1.0 / t13;
    v[1][0] = t14;
    let t15 = 0.00000000000000000e0;
    v[1][1] = t15;
    let t16 = 0.00000000000000000e0;
    v[2][0] = t16;
    let t17 = 0.00000000000000000e0;
    v[2][1] = t17;
    let t18 = 0.00000000000000000e0;
    v[3][0] = t18;
    let t19 = params[2];
    let t20 = params[0];
    let t21 = -t19;
    let t22 = t20 + t21;
    let t23 = t22.sin();
    let t24 = params[7];
    let t25 = t23.powf(2.00000000000000000e0);
    let t26 = params[6];
    let t27 = t24 * t25;
    let t28 = params[9];
    let t29 = t26 + t27;
    let t30 = t28 * t29;
    let t31 = 1.0 / t30;
    v[3][1] = t31;
    vec_to_dmat(&v)
}
