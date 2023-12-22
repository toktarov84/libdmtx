/**
 * libdmtx - Data Matrix Encoding/Decoding Library
 * Copyright 2008, 2009 Mike Laughton. All rights reserved.
 * Copyright 2012-2016 Vadim A. Misbakh-Soloviov. All rights reserved.
 * Copyright 2016 Tim Zaman. All rights reserved.
 *
 * See LICENSE file in the main project directory for full
 * terms of use and distribution.
 *
 * Contact:
 * Vadim A. Misbakh-Soloviov <dmtx@mva.name>
 * Mike Laughton <mike@dragonflylogic.com>
 *
 * \file dmtxregion.c
 * \brief Обнаружение областей штрих-кода
 */

#define DMTX_HOUGH_RES 180

/**
 * Создайте копию существующей структуры региона
 * \param  None
 * \return Инициализированная структура DmtxRegion
 */
extern DmtxRegion *
dmtxRegionCreate(DmtxRegion *reg)
{
   DmtxRegion *regCopy;

   regCopy = (DmtxRegion *)malloc(sizeof(DmtxRegion));
   if(regCopy == NULL)
      return NULL;

   memcpy(regCopy, reg, sizeof(DmtxRegion));

   return regCopy;
}

/**
 * Уничтожает объект области распознавания
 * \param  reg
 * \return void
 */
extern DmtxPassFail
dmtxRegionDestroy(DmtxRegion **reg)
{
   if(reg == NULL || *reg == NULL)
      return DmtxFail;

   free(*reg);

   *reg = NULL;

   return DmtxPass;
}

/**
 * Находит следующий регион со штрих-кодом
 * dec - Указатель на информационную структуру DmtxDecode
 * timeout - Указатель на время ожидания (NULL, если нет)
 * \return Обнаруженная область (если найдена)
 */
extern DmtxRegion *
dmtxRegionFindNext(DmtxDecode *dec, DmtxTime *timeout)
{
   int locStatus;
   DmtxPixelLoc loc;
   DmtxRegion   *reg;
   
   /* Продолжать, пока мы не найдем нужный регион или пока у нас не кончатся шансы */
   for(;;) {
      locStatus = PopGridLocation(&(dec->grid), &loc);
      if(locStatus == DmtxRangeEnd)
         break;

      /* Сканируйте местоположение на наличие действительного региона штрих-кода */
      reg = dmtxRegionScanPixel(dec, loc.X, loc.Y);
      if(reg != NULL)
         return reg;

      /* Не хватило времени? */
      if(timeout != NULL && dmtxTimeExceeded(*timeout))
         break;
   }

   return NULL;
}

/**
 * сканирование отдельного пикселя на наличие края штрих-кода
 * dec - указатель на информационную структуру DmtxDecode
 * loc - расположение пикселя
 * \return обнаруженную область (если таковая имеется)
 */
extern DmtxRegion *
dmtxRegionScanPixel(DmtxDecode *dec, int x, int y)
{
   unsigned char *cache;
   DmtxRegion reg;
   DmtxPointFlow flowBegin;
   DmtxPixelLoc loc;

   loc.X = x;
   loc.Y = y;

   // Кэширует текущую точку для последующего использования
   cache = dmtxDecodeGetCache(dec, loc.X, loc.Y);
   if(cache == NULL)
      return NULL;

   if((int)(*cache & 0x80) != 0x00)
      return NULL;

   /* Проверьте наличие любой разумной границы в этом месте */
   flowBegin = MatrixRegionSeekEdge(dec, loc);
   if(flowBegin.mag < (int)(dec->edgeThresh * 7.65 + 0.5))
      return NULL;

   memset(&reg, 0x00, sizeof(DmtxRegion));

   /* Определение ориентации штрих-кода */
   if(MatrixRegionOrientation(dec, &reg, flowBegin) == DmtxFail)
      return NULL;
   if(dmtxRegionUpdateXfrms(dec, &reg) == DmtxFail)
      return NULL;

   /**
    * Без MatrixRegionAlignCalibEdge не падает,
    * коды распознаёт, не замечено лучшее нахождение регионов.
    */
   
   /* Определите верхний край */
   if(MatrixRegionAlignCalibEdge(dec, &reg, DmtxEdgeTop) == DmtxFail)
      return NULL;
   if(dmtxRegionUpdateXfrms(dec, &reg) == DmtxFail)
      return NULL;

   /* Определите правый край */
   if(MatrixRegionAlignCalibEdge(dec, &reg, DmtxEdgeRight) == DmtxFail)
      return NULL;
   if(dmtxRegionUpdateXfrms(dec, &reg) == DmtxFail)
      return NULL;

   CALLBACK_MATRIX(&reg);

   /* Рассчитайте наиболее подходящий размер символа */
   if(MatrixRegionFindSize(dec, &reg) == DmtxFail)
      return NULL;

   /* Найдена допустимая область матрицы */
   return dmtxRegionCreate(&reg);
}

/**
 *
 *
 */
static DmtxPointFlow
MatrixRegionSeekEdge(DmtxDecode *dec, DmtxPixelLoc loc)
{
   int i;
   int strongIdx;
   int channelCount;
   DmtxPointFlow flow, flowPlane[3];
   DmtxPointFlow flowPos, flowPosBack;
   DmtxPointFlow flowNeg, flowNegBack;

   channelCount = dec->image->channelCount;

   /* Определите, какой из цветов - красный, зеленый или синий - имеет наиболее сильное преимущество */
   strongIdx = 0;
   for(i = 0; i < channelCount; i++) {
      flowPlane[i] = GetPointFlow(dec, i, loc, dmtxNeighborNone);
      if(i > 0 && flowPlane[i].mag > flowPlane[strongIdx].mag)
         strongIdx = i;
   }

   if(flowPlane[strongIdx].mag < 10)
      return dmtxBlankEdge;

   flow = flowPlane[strongIdx];

   flowPos = FindStrongestNeighbor(dec, flow, +1);
   flowNeg = FindStrongestNeighbor(dec, flow, -1);
   if(flowPos.mag != 0 && flowNeg.mag != 0) {
      flowPosBack = FindStrongestNeighbor(dec, flowPos, -1);
      flowNegBack = FindStrongestNeighbor(dec, flowNeg, +1);
      if(flowPos.arrive == (flowPosBack.arrive+4)%8 &&
            flowNeg.arrive == (flowNegBack.arrive+4)%8) {
         flow.arrive = dmtxNeighborNone;
         // CALLBACK_POINT_PLOT(flow.loc, 1, 1, 1);
         return flow;
      }
   }

   return dmtxBlankEdge;
}

/**
 *
 *
 */
static DmtxPassFail
MatrixRegionOrientation(DmtxDecode *dec, DmtxRegion *reg, DmtxPointFlow begin)
{
   int cross;
   int minArea;
   int scale;
   int symbolShape;
   int maxDiagonal;
   DmtxPassFail err;
   DmtxBestLine line1x, line2x;
   DmtxBestLine line2n, line2p;
   DmtxFollow fTmp;

   if(dec->sizeIdxExpected == DmtxSymbolSquareAuto ||
         (dec->sizeIdxExpected >= DmtxSymbol10x10 &&
         dec->sizeIdxExpected <= DmtxSymbol144x144))
      symbolShape = DmtxSymbolSquareAuto;
   else if(dec->sizeIdxExpected == DmtxSymbolRectAuto ||
         (dec->sizeIdxExpected >= DmtxSymbol8x18 &&
         dec->sizeIdxExpected <= DmtxSymbol16x48))
      symbolShape = DmtxSymbolRectAuto;
   else
      symbolShape = DmtxSymbolShapeAuto;

   if(dec->edgeMax != DmtxUndefined) {
      if(symbolShape == DmtxSymbolRectAuto)
         maxDiagonal = (int)(1.23 * dec->edgeMax + 0.5); /* sqrt(5/4) + 10% */
      else
         maxDiagonal = (int)(1.56 * dec->edgeMax + 0.5); /* sqrt(2) + 10% */
   }
   else {
      maxDiagonal = DmtxUndefined;
   }

   /* Следуйте до конца в обоих направлениях */
   err = TrailBlazeContinuous(dec, reg, begin, maxDiagonal);
   if(err == DmtxFail || reg->stepsTotal < 40) {
      TrailClear(dec, reg, 0x40);
      return DmtxFail;
   }

   /* Отфильтруйте регионы-кандидаты, которые меньше, чем ожидалось */
   if(dec->edgeMin != DmtxUndefined) {
      scale = dmtxDecodeGetProp(dec, DmtxPropScale);

      if(symbolShape == DmtxSymbolSquareAuto)
         minArea = (dec->edgeMin * dec->edgeMin)/(scale * scale);
      else
         minArea = (2 * dec->edgeMin * dec->edgeMin)/(scale * scale);

      if((reg->boundMax.X - reg->boundMin.X) * (reg->boundMax.Y - reg->boundMin.Y) < minArea) {
         TrailClear(dec, reg, 0x40);
         return DmtxFail;
      }
   }

   line1x = FindBestSolidLine(dec, reg, 0, 0, +1, DmtxUndefined);
   if(line1x.mag < 5) {
      TrailClear(dec, reg, 0x40);
      return DmtxFail;
   }

   err = FindTravelLimits(dec, reg, &line1x);
   if(line1x.distSq < 100 || line1x.devn * 10 >= sqrt((double)line1x.distSq)) {
      TrailClear(dec, reg, 0x40);
      return DmtxFail;
   }
   assert(line1x.stepPos >= line1x.stepNeg);

   fTmp = FollowSeek(dec, reg, line1x.stepPos + 5);
   line2p = FindBestSolidLine(dec, reg, fTmp.step, line1x.stepNeg, +1, line1x.angle);

   fTmp = FollowSeek(dec, reg, line1x.stepNeg - 5);
   line2n = FindBestSolidLine(dec, reg, fTmp.step, line1x.stepPos, -1, line1x.angle);
   if(max(line2p.mag, line2n.mag) < 5)
      return DmtxFail;

   if(line2p.mag > line2n.mag) {
      line2x = line2p;
      err = FindTravelLimits(dec, reg, &line2x);
      if(line2x.distSq < 100 || line2x.devn * 10 >= sqrt((double)line2x.distSq))
         return DmtxFail;

      cross = ((line1x.locPos.X - line1x.locNeg.X) * (line2x.locPos.Y - line2x.locNeg.Y)) -
            ((line1x.locPos.Y - line1x.locNeg.Y) * (line2x.locPos.X - line2x.locNeg.X));
      if(cross > 0) {
         /* Состояние 2 */
         reg->polarity = +1;
         reg->locR = line2x.locPos;
         reg->stepR = line2x.stepPos;
         reg->locT = line1x.locNeg;
         reg->stepT = line1x.stepNeg;
         reg->leftLoc = line1x.locBeg;
         reg->leftAngle = line1x.angle;
         reg->bottomLoc = line2x.locBeg;
         reg->bottomAngle = line2x.angle;
         reg->leftLine = line1x;
         reg->bottomLine = line2x;
      }
      else {
         /* Состояние 3 */
         reg->polarity = -1;
         reg->locR = line1x.locNeg;
         reg->stepR = line1x.stepNeg;
         reg->locT = line2x.locPos;
         reg->stepT = line2x.stepPos;
         reg->leftLoc = line2x.locBeg;
         reg->leftAngle = line2x.angle;
         reg->bottomLoc = line1x.locBeg;
         reg->bottomAngle = line1x.angle;
         reg->leftLine = line2x;
         reg->bottomLine = line1x;
      }
   }
   else {
      line2x = line2n;
      err = FindTravelLimits(dec, reg, &line2x);
      if(line2x.distSq < 100 || line2x.devn / sqrt((double)line2x.distSq) >= 0.1)
         return DmtxFail;

      cross = ((line1x.locNeg.X - line1x.locPos.X) * (line2x.locNeg.Y - line2x.locPos.Y)) -
            ((line1x.locNeg.Y - line1x.locPos.Y) * (line2x.locNeg.X - line2x.locPos.X));
      if(cross > 0) {
         /* Состояние 1 */
         reg->polarity = -1;
         reg->locR = line2x.locNeg;
         reg->stepR = line2x.stepNeg;
         reg->locT = line1x.locPos;
         reg->stepT = line1x.stepPos;
         reg->leftLoc = line1x.locBeg;
         reg->leftAngle = line1x.angle;
         reg->bottomLoc = line2x.locBeg;
         reg->bottomAngle = line2x.angle;
         reg->leftLine = line1x;
         reg->bottomLine = line2x;
      }
      else {
         /* Состояние 4 */
         reg->polarity = +1;
         reg->locR = line1x.locPos;
         reg->stepR = line1x.stepPos;
         reg->locT = line2x.locNeg;
         reg->stepT = line2x.stepNeg;
         reg->leftLoc = line2x.locBeg;
         reg->leftAngle = line2x.angle;
         reg->bottomLoc = line1x.locBeg;
         reg->bottomAngle = line1x.angle;
         reg->leftLine = line2x;
         reg->bottomLine = line1x;
      }
   }
/* CALLBACK_POINT_PLOT(reg->locR, 2, 1, 1);
   CALLBACK_POINT_PLOT(reg->locT, 2, 1, 1); */

   reg->leftKnown = reg->bottomKnown = 1;

   return DmtxPass;
}

/**
 *
 *
 */
static long
DistanceSquared(DmtxPixelLoc a, DmtxPixelLoc b)
{
   long xDelta, yDelta;

   xDelta = a.X - b.X;
   yDelta = a.Y - b.Y;

   return (xDelta * xDelta) + (yDelta * yDelta);
}

/**
 *
 *
 */
extern DmtxPassFail
dmtxRegionUpdateCorners(DmtxDecode *dec, DmtxRegion *reg, DmtxVector2 p00,
      DmtxVector2 p10, DmtxVector2 p11, DmtxVector2 p01)
{
   double xMax, yMax;
   double tx, ty, phi, shx, scx, scy, skx, sky;
   double dimOT, dimOR, dimTX, dimRX, ratio;
   DmtxVector2 vOT, vOR, vTX, vRX, vTmp;
   DmtxMatrix3 m, mtxy, mphi, mshx, mscx, mscy, mscxy, msky, mskx;

   xMax = (double)(dmtxDecodeGetProp(dec, DmtxPropWidth) - 1);
   yMax = (double)(dmtxDecodeGetProp(dec, DmtxPropHeight) - 1);

   if(p00.X < 0.0 || p00.Y < 0.0 || p00.X > xMax || p00.Y > yMax ||
         p01.X < 0.0 || p01.Y < 0.0 || p01.X > xMax || p01.Y > yMax ||
         p10.X < 0.0 || p10.Y < 0.0 || p10.X > xMax || p10.Y > yMax)
      return DmtxFail;

   dimOT = dmtxVector2Mag(dmtxVector2Sub(&vOT, &p01, &p00)); /* XXX мог бы использовать MagSquared() */
   dimOR = dmtxVector2Mag(dmtxVector2Sub(&vOR, &p10, &p00));
   dimTX = dmtxVector2Mag(dmtxVector2Sub(&vTX, &p11, &p01));
   dimRX = dmtxVector2Mag(dmtxVector2Sub(&vRX, &p11, &p10));

   /* Убедитесь, что боковые стороны достаточно длинные */
   if(dimOT <= 8.0 || dimOR <= 8.0 || dimTX <= 8.0 || dimRX <= 8.0)
      return DmtxFail;

   /* Убедитесь, что 4 угла образуют достаточно толстый четырехугольник */
   ratio = dimOT / dimRX;
   if(ratio <= 0.5 || ratio >= 2.0)
      return DmtxFail;

   ratio = dimOR / dimTX;
   if(ratio <= 0.5 || ratio >= 2.0)
      return DmtxFail;

   /* Убедитесь, что это не форма галстука-бабочки */
   if(dmtxVector2Cross(&vOR, &vRX) <= 0.0 ||
         dmtxVector2Cross(&vOT, &vTX) >= 0.0)
      return DmtxFail;

   if(RightAngleTrueness(p00, p10, p11, M_PI_2) <= dec->squareDevn)
      return DmtxFail;
   if(RightAngleTrueness(p10, p11, p01, M_PI_2) <= dec->squareDevn)
      return DmtxFail;

   /* Вычислять значения, необходимые для преобразований */
   tx = -1 * p00.X;
   ty = -1 * p00.Y;
   dmtxMatrix3Translate(mtxy, tx, ty);

   phi = atan2(vOT.X, vOT.Y);
   dmtxMatrix3Rotate(mphi, phi);
   dmtxMatrix3Multiply(m, mtxy, mphi);

   dmtxMatrix3VMultiply(&vTmp, &p10, m);
   shx = -vTmp.Y / vTmp.X;
   dmtxMatrix3Shear(mshx, 0.0, shx);
   dmtxMatrix3MultiplyBy(m, mshx);

   scx = 1.0/vTmp.X;
   dmtxMatrix3Scale(mscx, scx, 1.0);
   dmtxMatrix3MultiplyBy(m, mscx);

   dmtxMatrix3VMultiply(&vTmp, &p11, m);
   scy = 1.0/vTmp.Y;
   dmtxMatrix3Scale(mscy, 1.0, scy);
   dmtxMatrix3MultiplyBy(m, mscy);

   dmtxMatrix3VMultiply(&vTmp, &p11, m);
   skx = vTmp.X;
   dmtxMatrix3LineSkewSide(mskx, 1.0, skx, 1.0);
   dmtxMatrix3MultiplyBy(m, mskx);

   dmtxMatrix3VMultiply(&vTmp, &p01, m);
   sky = vTmp.Y;
   dmtxMatrix3LineSkewTop(msky, sky, 1.0, 1.0);
   dmtxMatrix3Multiply(reg->raw2fit, m, msky);

   /* Создайте обратную матрицу обратным способом (избегайте прямой инверсии матрицы) */
   dmtxMatrix3LineSkewTopInv(msky, sky, 1.0, 1.0);
   dmtxMatrix3LineSkewSideInv(mskx, 1.0, skx, 1.0);
   dmtxMatrix3Multiply(m, msky, mskx);

   dmtxMatrix3Scale(mscxy, 1.0/scx, 1.0/scy);
   dmtxMatrix3MultiplyBy(m, mscxy);

   dmtxMatrix3Shear(mshx, 0.0, -shx);
   dmtxMatrix3MultiplyBy(m, mshx);

   dmtxMatrix3Rotate(mphi, -phi);
   dmtxMatrix3MultiplyBy(m, mphi);

   dmtxMatrix3Translate(mtxy, -tx, -ty);
   dmtxMatrix3Multiply(reg->fit2raw, m, mtxy);

   return DmtxPass;
}

/**
 *
 *
 */
extern DmtxPassFail
dmtxRegionUpdateXfrms(DmtxDecode *dec, DmtxRegion *reg)
{
   double radians;
   DmtxRay2 rLeft, rBottom, rTop, rRight;
   DmtxVector2 p00, p10, p11, p01;

   assert(reg->leftKnown != 0 && reg->bottomKnown != 0);

   /* Построить луч, представляющий левый край */
   rLeft.p.X = (double)reg->leftLoc.X;
   rLeft.p.Y = (double)reg->leftLoc.Y;
   radians = reg->leftAngle * (M_PI/DMTX_HOUGH_RES);
   rLeft.v.X = cos(radians);
   rLeft.v.Y = sin(radians);
   rLeft.tMin = 0.0;
   rLeft.tMax = dmtxVector2Norm(&rLeft.v);

   /* Построить луч, представляющий нижний край */
   rBottom.p.X = (double)reg->bottomLoc.X;
   rBottom.p.Y = (double)reg->bottomLoc.Y;
   radians = reg->bottomAngle * (M_PI/DMTX_HOUGH_RES);
   rBottom.v.X = cos(radians);
   rBottom.v.Y = sin(radians);
   rBottom.tMin = 0.0;
   rBottom.tMax = dmtxVector2Norm(&rBottom.v);

   /* Построить луч, представляющий верхний край */
   if(reg->topKnown != 0) {
      rTop.p.X = (double)reg->topLoc.X;
      rTop.p.Y = (double)reg->topLoc.Y;
      radians = reg->topAngle * (M_PI/DMTX_HOUGH_RES);
      rTop.v.X = cos(radians);
      rTop.v.Y = sin(radians);
      rTop.tMin = 0.0;
      rTop.tMax = dmtxVector2Norm(&rTop.v);
   }
   else {
      rTop.p.X = (double)reg->locT.X;
      rTop.p.Y = (double)reg->locT.Y;
      radians = reg->bottomAngle * (M_PI/DMTX_HOUGH_RES);
      rTop.v.X = cos(radians);
      rTop.v.Y = sin(radians);
      rTop.tMin = 0.0;
      rTop.tMax = rBottom.tMax;
   }

   /* Построить луч, представляющий правый край */
   if(reg->rightKnown != 0) {
      rRight.p.X = (double)reg->rightLoc.X;
      rRight.p.Y = (double)reg->rightLoc.Y;
      radians = reg->rightAngle * (M_PI/DMTX_HOUGH_RES);
      rRight.v.X = cos(radians);
      rRight.v.Y = sin(radians);
      rRight.tMin = 0.0;
      rRight.tMax = dmtxVector2Norm(&rRight.v);
   }
   else {
      rRight.p.X = (double)reg->locR.X;
      rRight.p.Y = (double)reg->locR.Y;
      radians = reg->leftAngle * (M_PI/DMTX_HOUGH_RES);
      rRight.v.X = cos(radians);
      rRight.v.Y = sin(radians);
      rRight.tMin = 0.0;
      rRight.tMax = rLeft.tMax;
   }

   /* Вычислите 4 угла, реальных или воображаемых */
   if(dmtxRay2Intersect(&p00, &rLeft, &rBottom) == DmtxFail)
      return DmtxFail;

   if(dmtxRay2Intersect(&p10, &rBottom, &rRight) == DmtxFail)
      return DmtxFail;

   if(dmtxRay2Intersect(&p11, &rRight, &rTop) == DmtxFail)
      return DmtxFail;

   if(dmtxRay2Intersect(&p01, &rTop, &rLeft) == DmtxFail)
      return DmtxFail;

   if(dmtxRegionUpdateCorners(dec, reg, p00, p10, p11, p01) != DmtxPass)
      return DmtxFail;

   return DmtxPass;
}

/**
 *
 *
 */
static double
RightAngleTrueness(DmtxVector2 c0, DmtxVector2 c1, DmtxVector2 c2, double angle)
{
   DmtxVector2 vA, vB;
   DmtxMatrix3 m;

   dmtxVector2Norm(dmtxVector2Sub(&vA, &c0, &c1));
   dmtxVector2Norm(dmtxVector2Sub(&vB, &c2, &c1));

   dmtxMatrix3Rotate(m, angle);
   dmtxMatrix3VMultiplyBy(&vB, m);

   return dmtxVector2Dot(&vA, &vB);
}

/**
 * \brief  Считывание цвета расположения модуля матрицы данных
 * \param  dec
 * \param  reg
 * \param  symbolRow
 * \param  symbolCol
 * \param  sizeIdx
 * \return Усредненный цвет модуля
 */
static int
ReadModuleColor(DmtxDecode *dec, DmtxRegion *reg, int symbolRow, int symbolCol,
      int sizeIdx, int colorPlane)
{
   int i;
   int symbolRows, symbolCols;
   int color, colorTmp;
   double sampleX[] = { 0.5, 0.4, 0.5, 0.6, 0.5 };
   double sampleY[] = { 0.5, 0.5, 0.4, 0.5, 0.6 };
   DmtxVector2 p;

   symbolRows = dmtxGetSymbolAttribute(DmtxSymAttribSymbolRows, sizeIdx);
   symbolCols = dmtxGetSymbolAttribute(DmtxSymAttribSymbolCols, sizeIdx);

   color = 0;
   for(i = 0; i < 5; i++) {

      p.X = (1.0/symbolCols) * (symbolCol + sampleX[i]);
      p.Y = (1.0/symbolRows) * (symbolRow + sampleY[i]);

      dmtxMatrix3VMultiplyBy(&p, reg->fit2raw);

      //fprintf(stdout, "%dx%d\n", (int)(p.X + 0.5), (int)(p.Y + 0.5));

      dmtxDecodeGetPixelValue(dec, (int)(p.X + 0.5), (int)(p.Y + 0.5),
            colorPlane, &colorTmp);
      color += colorTmp;
   }
   //fprintf(stdout, "\n");
   return color/5;
}

/**
 * \brief  Определить размер штрих-кода, выраженный в модулях
 * \param  image
 * \param  reg
 * \return DmtxPass | DmtxFail
 */
static DmtxPassFail
MatrixRegionFindSize(DmtxDecode *dec, DmtxRegion *reg)
{
   int row, col;
   int sizeIdxBeg, sizeIdxEnd;
   int sizeIdx, bestSizeIdx;
   int symbolRows, symbolCols;
   int jumpCount, errors;
   int color;
   int colorOnAvg, bestColorOnAvg;
   int colorOffAvg, bestColorOffAvg;
   int contrast, bestContrast;
//   DmtxImage *img;

//   img = dec->image;
   bestSizeIdx = DmtxUndefined;
   bestContrast = 0;
   bestColorOnAvg = bestColorOffAvg = 0;

   if(dec->sizeIdxExpected == DmtxSymbolShapeAuto) {
      sizeIdxBeg = 0;
      sizeIdxEnd = DmtxSymbolSquareCount + DmtxSymbolRectCount;
   }
   else if(dec->sizeIdxExpected == DmtxSymbolSquareAuto) {
      sizeIdxBeg = 0;
      sizeIdxEnd = DmtxSymbolSquareCount;
   }
   else if(dec->sizeIdxExpected == DmtxSymbolRectAuto) {
      sizeIdxBeg = DmtxSymbolSquareCount;
      sizeIdxEnd = DmtxSymbolSquareCount + DmtxSymbolRectCount;
   }
   else {
      sizeIdxBeg = dec->sizeIdxExpected;
      sizeIdxEnd = dec->sizeIdxExpected + 1;
   }

   /* Протестируйте каждый размер штрих-кода, чтобы найти наилучший контраст в калибровочных модулях */
   for(sizeIdx = sizeIdxBeg; sizeIdx < sizeIdxEnd; sizeIdx++) {

      symbolRows = dmtxGetSymbolAttribute(DmtxSymAttribSymbolRows, sizeIdx);
      symbolCols = dmtxGetSymbolAttribute(DmtxSymAttribSymbolCols, sizeIdx);
      colorOnAvg = colorOffAvg = 0;

      /* Суммируйте цвета модуля вдоль горизонтальной калибровочной полосы */
      row = symbolRows - 1;
      for(col = 0; col < symbolCols; col++) {
         color = ReadModuleColor(dec, reg, row, col, sizeIdx, reg->flowBegin.plane);
         if((col & 0x01) != 0x00)
            colorOffAvg += color;
         else
            colorOnAvg += color;
      }

      /* Суммируйте цвета модуля вдоль вертикальной калибровочной полосы */
      col = symbolCols - 1;
      for(row = 0; row < symbolRows; row++) {
         color = ReadModuleColor(dec, reg, row, col, sizeIdx, reg->flowBegin.plane);
         if((row & 0x01) != 0x00)
            colorOffAvg += color;
         else
            colorOnAvg += color;
      }

      colorOnAvg = (colorOnAvg * 2)/(symbolRows + symbolCols);
      colorOffAvg = (colorOffAvg * 2)/(symbolRows + symbolCols);

      contrast = abs(colorOnAvg - colorOffAvg);
      if(contrast < 5)
         continue;

      if(contrast > bestContrast) {
         bestContrast = contrast;
         bestSizeIdx = sizeIdx;
         bestColorOnAvg = colorOnAvg;
         bestColorOffAvg = colorOffAvg;
      }
   }

   /* Если ни один из размеров не обеспечил приемлемого контраста, то считайте, что все завершено */
   // if(bestSizeIdx == DmtxUndefined || bestContrast < 20)
   if(bestSizeIdx == DmtxUndefined || bestContrast < 5)
      return DmtxFail;

   reg->sizeIdx = bestSizeIdx;
   reg->onColor = bestColorOnAvg;
   reg->offColor = bestColorOffAvg;

   reg->symbolRows = dmtxGetSymbolAttribute(DmtxSymAttribSymbolRows, reg->sizeIdx);
   reg->symbolCols = dmtxGetSymbolAttribute(DmtxSymAttribSymbolCols, reg->sizeIdx);
   reg->mappingRows = dmtxGetSymbolAttribute(DmtxSymAttribMappingMatrixRows, reg->sizeIdx);
   reg->mappingCols = dmtxGetSymbolAttribute(DmtxSymAttribMappingMatrixCols, reg->sizeIdx);

   /* Счетчик перемещается по горизонтальной калибровочной линейке для проверки размера(sizeIdx). */
   jumpCount = CountJumpTally(dec, reg, 0, reg->symbolRows - 1, DmtxDirRight);
   errors = abs(1 + jumpCount - reg->symbolCols);
   if(jumpCount < 0 || errors > 6)
      return DmtxFail;

   /* Счетчик перемещается по вертикальной шкале калибровки для проверки размера(sizeIdx). */
   jumpCount = CountJumpTally(dec, reg, reg->symbolCols - 1, 0, DmtxDirUp);
   errors = abs(1 + jumpCount - reg->symbolRows);
   if(jumpCount < 0 || errors > 6)
      return DmtxFail;

   /* Подсчет переходит на горизонтальную панель поиска для проверки размера(sizeIdx). */
   errors = CountJumpTally(dec, reg, 0, 0, DmtxDirRight);
   if(jumpCount < 0 || errors > 6)
      return DmtxFail;

   /* Подсчет переходит на вертикальную панель поиска, чтобы проверить размер(sizeIdx). */
   errors = CountJumpTally(dec, reg, 0, 0, DmtxDirUp);
   if(errors < 0 || errors > 6)
     return DmtxFail;

   /* Подсчет переходит на окружающие пробелы, иначе произойдет сбой */
   errors = CountJumpTally(dec, reg, 0, -1, DmtxDirRight);
   if(errors < 0 || errors > 3)
     return DmtxFail;

   errors = CountJumpTally(dec, reg, -1, 0, DmtxDirUp);
   if(errors < 0 || errors > 3)
     return DmtxFail;

   errors = CountJumpTally(dec, reg, 0, reg->symbolRows, DmtxDirRight);
   if(errors < 0 || errors > 3)
     return DmtxFail;

   errors = CountJumpTally(dec, reg, reg->symbolCols, 0, DmtxDirUp);
   if(errors < 0 || errors > 3)
     return DmtxFail;

   return DmtxPass;
}

/**
 * \brief  Подсчитайте количество переходов между светлым и темным
 * \param  img
 * \param  reg
 * \param  xStart
 * \param  yStart
 * \param  dir
 * \return Количество прыжков
 */
static int
CountJumpTally(DmtxDecode *dec, DmtxRegion *reg, int xStart, int yStart, DmtxDirection dir)
{
   int x, xInc = 0;
   int y, yInc = 0;
   int state = DmtxModuleOn;
   int jumpCount = 0;
   int jumpThreshold;
   int tModule, tPrev;
   int darkOnLight;
   int color;

   assert(xStart == 0 || yStart == 0);
   assert(dir == DmtxDirRight || dir == DmtxDirUp);

   if(dir == DmtxDirRight)
      xInc = 1;
   else
      yInc = 1;

   if(xStart == -1 || xStart == reg->symbolCols ||
         yStart == -1 || yStart == reg->symbolRows)
      state = DmtxModuleOff;

   darkOnLight = (int)(reg->offColor > reg->onColor);
   jumpThreshold = abs((int)(0.4 * (reg->onColor - reg->offColor) + 0.5));
   color = ReadModuleColor(dec, reg, yStart, xStart, reg->sizeIdx, reg->flowBegin.plane);
   tModule = (darkOnLight) ? reg->offColor - color : color - reg->offColor;

   for(x = xStart + xInc, y = yStart + yInc;
         (dir == DmtxDirRight && x < reg->symbolCols) ||
         (dir == DmtxDirUp && y < reg->symbolRows);
         x += xInc, y += yInc) {

      tPrev = tModule;
      color = ReadModuleColor(dec, reg, y, x, reg->sizeIdx, reg->flowBegin.plane);
      tModule = (darkOnLight) ? reg->offColor - color : color - reg->offColor;

      if(state == DmtxModuleOff) {
         if(tModule > tPrev + jumpThreshold) {
            jumpCount++;
            state = DmtxModuleOn;
         }
      }
      else {
         if(tModule < tPrev - jumpThreshold) {
            jumpCount++;
            state = DmtxModuleOff;
         }
      }
   }

   return jumpCount;
}

/**
 *
 *
 */
static DmtxPointFlow
GetPointFlow(DmtxDecode *dec, int colorPlane, DmtxPixelLoc loc, int arrive)
{
   static const int coefficient[] = {  0,  1,  2,  1,  0, -1, -2, -1 };
   int err;
   int patternIdx, coefficientIdx;
   int compass, compassMax;
   int mag[4] = { 0 };
   int xAdjust, yAdjust;
   int color, colorPattern[8];
   DmtxPointFlow flow;

   for(patternIdx = 0; patternIdx < 8; patternIdx++) {
      xAdjust = loc.X + dmtxPatternX[patternIdx];
      yAdjust = loc.Y + dmtxPatternY[patternIdx];
      err = dmtxDecodeGetPixelValue(dec, xAdjust, yAdjust, colorPlane,
            &colorPattern[patternIdx]);
      if(err == DmtxFail)
         return dmtxBlankEdge;
   }

   /* Вычислите интенсивность потока этого пикселя для каждого направления (-45, 0, 45, 90) */
   compassMax = 0;
   for(compass = 0; compass < 4; compass++) {

      /* Добавьте часть из каждой позиции в шаблоне матрицы свертки */
      for(patternIdx = 0; patternIdx < 8; patternIdx++) {

         coefficientIdx = (patternIdx - compass + 8) % 8;
         if(coefficient[coefficientIdx] == 0)
            continue;

         color = colorPattern[patternIdx];

         switch(coefficient[coefficientIdx]) {
            case 2:
               mag[compass] += color;
               /* Провалиться сквозь */
            case 1:
               mag[compass] += color;
               break;
            case -2:
               mag[compass] -= color;
               /* Провалиться сквозь */
            case -1:
               mag[compass] -= color;
               break;
         }
      }

      /* Определите самый сильный поток по компасу */
      if(compass != 0 && abs(mag[compass]) > abs(mag[compassMax]))
         compassMax = compass;
   }

   /* Преобразуйте подписанное направление компаса в уникальные направления потока (0-7) */
   flow.plane = colorPlane;
   flow.arrive = arrive;
   flow.depart = (mag[compassMax] > 0) ? compassMax + 4 : compassMax;
   flow.mag = abs(mag[compassMax]);
   flow.loc = loc;

   return flow;
}

/**
 *
 *
 */
static DmtxPointFlow
FindStrongestNeighbor(DmtxDecode *dec, DmtxPointFlow center, int sign)
{
   int i;
   int strongIdx;
   int attempt, attemptDiff;
   int occupied;
   unsigned char *cache;
   DmtxPixelLoc loc;
   DmtxPointFlow flow[8];

   attempt = (sign < 0) ? center.depart : (center.depart+4)%8;

   occupied = 0;
   strongIdx = DmtxUndefined;
   for(i = 0; i < 8; i++) {

      loc.X = center.loc.X + dmtxPatternX[i];
      loc.Y = center.loc.Y + dmtxPatternY[i];

      cache = dmtxDecodeGetCache(dec, loc.X, loc.Y);
      if(cache == NULL)
         continue;

      if((int)(*cache & 0x80) != 0x00) {
         if(++occupied > 2)
            return dmtxBlankEdge;
         else
            continue;
      }

      attemptDiff = abs(attempt - i);
      if(attemptDiff > 4)
         attemptDiff = 8 - attemptDiff;
      if(attemptDiff > 1)
         continue;

      flow[i] = GetPointFlow(dec, center.plane, loc, i);

      if(strongIdx == DmtxUndefined || flow[i].mag > flow[strongIdx].mag ||
            (flow[i].mag == flow[strongIdx].mag && ((i & 0x01) != 0))) {
         strongIdx = i;
      }
   }

   return (strongIdx == DmtxUndefined) ? dmtxBlankEdge : flow[strongIdx];
}

/**
 *
 *
 */
static DmtxFollow
FollowSeek(DmtxDecode *dec, DmtxRegion *reg, int seek)
{
   int i;
   int sign;
   DmtxFollow follow;

   follow.loc = reg->flowBegin.loc;
   follow.step = 0;
   follow.ptr = dmtxDecodeGetCache(dec, follow.loc.X, follow.loc.Y);
   assert(follow.ptr != NULL);
   follow.neighbor = *follow.ptr;

   sign = (seek > 0) ? +1 : -1;
   for(i = 0; i != seek; i += sign) {
      follow = FollowStep(dec, reg, follow, sign);
      assert(follow.ptr != NULL);
      assert(abs(follow.step) <= reg->stepsTotal);
   }

   return follow;
}

/**
 *
 *
 */
static DmtxFollow
FollowSeekLoc(DmtxDecode *dec, DmtxPixelLoc loc)
{
   DmtxFollow follow;

   follow.loc = loc;
   follow.step = 0;
   follow.ptr = dmtxDecodeGetCache(dec, follow.loc.X, follow.loc.Y);
   assert(follow.ptr != NULL);
   follow.neighbor = *follow.ptr;

   return follow;
}


/**
 *
 *
 */
static DmtxFollow
FollowStep(DmtxDecode *dec, DmtxRegion *reg, DmtxFollow followBeg, int sign)
{
   int patternIdx;
   int stepMod;
   int factor;
   DmtxFollow follow;

   assert(abs(sign) == 1);
   assert((int)(followBeg.neighbor & 0x40) != 0x00);

   factor = reg->stepsTotal + 1;
   if(sign > 0)
      stepMod = (factor + (followBeg.step % factor)) % factor;
   else
      stepMod = (factor - (followBeg.step % factor)) % factor;

   /* Конец позитивного следа - волшебный прыжок */
   if(sign > 0 && stepMod == reg->jumpToNeg) {
      follow.loc = reg->finalNeg;
   }
   /* Конец негативного следа - волшебный прыжок */
   else if(sign < 0 && stepMod == reg->jumpToPos) {
      follow.loc = reg->finalPos;
   }
   /* Отслеживание продолжается - обычный прыжок */
   else {
      patternIdx = (sign < 0) ? followBeg.neighbor & 0x07 : ((followBeg.neighbor & 0x38) >> 3);
      follow.loc.X = followBeg.loc.X + dmtxPatternX[patternIdx];
      follow.loc.Y = followBeg.loc.Y + dmtxPatternY[patternIdx];
   }

   follow.step = followBeg.step + sign;
   follow.ptr = dmtxDecodeGetCache(dec, follow.loc.X, follow.loc.Y);
   assert(follow.ptr != NULL);
   follow.neighbor = *follow.ptr;

   return follow;
}

/**
 *
 *
 */
static DmtxFollow
FollowStep2(DmtxDecode *dec, DmtxFollow followBeg, int sign)
{
   int patternIdx;
   DmtxFollow follow;

   assert(abs(sign) == 1);
   assert((int)(followBeg.neighbor & 0x40) != 0x00);

   patternIdx = (sign < 0) ? followBeg.neighbor & 0x07 : ((followBeg.neighbor & 0x38) >> 3);
   follow.loc.X = followBeg.loc.X + dmtxPatternX[patternIdx];
   follow.loc.Y = followBeg.loc.Y + dmtxPatternY[patternIdx];

   follow.step = followBeg.step + sign;
   follow.ptr = dmtxDecodeGetCache(dec, follow.loc.X, follow.loc.Y);
   assert(follow.ptr != NULL);
   follow.neighbor = *follow.ptr;

   return follow;
}

/**
 * vaiiiooo
 * --------
 * 0x80 v = посещенный бит
 * 0x40 a = назначенный бит
 * 0x38 u = 3 биты указывают вверх по потоку 0-7
 * 0x07 d = 3 биты указывают вниз по потоку 0-7
 */
static DmtxPassFail
TrailBlazeContinuous(DmtxDecode *dec, DmtxRegion *reg, DmtxPointFlow flowBegin, int maxDiagonal)
{
   int posAssigns, negAssigns, clears;
   int sign;
   int steps;
   unsigned char *cache, *cacheNext, *cacheBeg;
   DmtxPointFlow flow, flowNext;
   DmtxPixelLoc boundMin, boundMax;

   boundMin = boundMax = flowBegin.loc;
   cacheBeg = dmtxDecodeGetCache(dec, flowBegin.loc.X, flowBegin.loc.Y);
   if(cacheBeg == NULL)
      return DmtxFail;
   *cacheBeg = (0x80 | 0x40); /* Отметьте местоположение как посещенное и назначенное */

   reg->flowBegin = flowBegin;

   posAssigns = negAssigns = 0;
   for(sign = 1; sign >= -1; sign -= 2) {

      flow = flowBegin;
      cache = cacheBeg;

      for(steps = 0; ; steps++) {

         if(maxDiagonal != DmtxUndefined && (boundMax.X - boundMin.X > maxDiagonal ||
               boundMax.Y - boundMin.Y > maxDiagonal))
            break;

         /* Найдите самого сильного подходящего соседа */
         flowNext = FindStrongestNeighbor(dec, flow, sign);
         if(flowNext.mag < 50)
            break;

         /* Получить местоположение кэша соседа */
         cacheNext = dmtxDecodeGetCache(dec, flowNext.loc.X, flowNext.loc.Y);
         if(cacheNext == NULL)
            break;
         assert(!(*cacheNext & 0x80));

         /* Отметьте отправление от текущего местоположения. Если течет вниз по течению
          * (sign < 0) вектор отправления здесь - это вектор прибытия
          * из следующего местоположения. Восходящий поток использует противоположное правило. */
         *cache |= (sign < 0) ? flowNext.arrive : flowNext.arrive << 3;

         /* Отметьте известное направление для следующего местоположения */
         /* При тестировании ниже по потоку (sign < 0) следующий подъем вверх по течению противоположен следующему прибытию */
         /* При тестировании вверх по течению (sign > 0) следующий спуск по течению противоположен следующему прибытию */
         *cacheNext = (sign < 0) ? (((flowNext.arrive + 4)%8) << 3) : ((flowNext.arrive + 4)%8);
         *cacheNext |= (0x80 | 0x40); /* Отметьте местоположение как посещенное и назначенное */
         if(sign > 0)
            posAssigns++;
         else
            negAssigns++;
         cache = cacheNext;
         flow = flowNext;

         if(flow.loc.X > boundMax.X)
            boundMax.X = flow.loc.X;
         else if(flow.loc.X < boundMin.X)
            boundMin.X = flow.loc.X;
         if(flow.loc.Y > boundMax.Y)
            boundMax.Y = flow.loc.Y;
         else if(flow.loc.Y < boundMin.Y)
            boundMin.Y = flow.loc.Y;

/*       CALLBACK_POINT_PLOT(flow.loc, (sign > 0) ? 2 : 3, 1, 2); */
      }

      if(sign > 0) {
         reg->finalPos = flow.loc;
         reg->jumpToNeg = steps;
      }
      else {
         reg->finalNeg = flow.loc;
         reg->jumpToPos = steps;
      }
   }
   reg->stepsTotal = reg->jumpToPos + reg->jumpToNeg;
   reg->boundMin = boundMin;
   reg->boundMax = boundMax;

   /* Очистить "посещенный" фрагмент от следа */
   clears = TrailClear(dec, reg, 0x80);
   assert(posAssigns + negAssigns == clears - 1);

   /* XXX приберись здесь ... избыточный тест, приведенный выше */
   if(maxDiagonal != DmtxUndefined && (boundMax.X - boundMin.X > maxDiagonal ||
         boundMax.Y - boundMin.Y > maxDiagonal))
      return DmtxFail;

   return DmtxPass;
}

/**
 * получает волокно(bresline) и следует за самым сильным соседом, если только это не связано с
 * растягивание волокон(bresline) по линии внутрь или назад (although back + outward is allowed).
 *
 */
static int
TrailBlazeGapped(DmtxDecode *dec, DmtxRegion *reg, DmtxBresLine line, int streamDir)
{
   unsigned char *beforeCache, *afterCache;
   DmtxBoolean onEdge;
   int distSq, distSqMax;
   int travel, outward;
   int xDiff, yDiff;
   int steps;
   int stepDir, dirMap[] = { 0, 1, 2, 7, 8, 3, 6, 5, 4 };
   DmtxPassFail err;
   DmtxPixelLoc beforeStep, afterStep;
   DmtxPointFlow flow, flowNext;
   DmtxPixelLoc loc0;
   int xStep, yStep;

   loc0 = line.loc;
   flow = GetPointFlow(dec, reg->flowBegin.plane, loc0, dmtxNeighborNone);
   distSqMax = (line.xDelta * line.xDelta) + (line.yDelta * line.yDelta);
   steps = 0;
   onEdge = DmtxTrue;

   beforeStep = loc0;
   beforeCache = dmtxDecodeGetCache(dec, loc0.X, loc0.Y);
   if(beforeCache == NULL)
      return DmtxFail;
   else
      *beforeCache = 0x00; /* вероятно, следует просто перезаписать одно направление */

   do {
      if(onEdge == DmtxTrue) {
         flowNext = FindStrongestNeighbor(dec, flow, streamDir);
         if(flowNext.mag == DmtxUndefined)
            break;

         err = BresLineGetStep(line, flowNext.loc, &travel, &outward);
         if (err == DmtxFail) { return DmtxFail; }

         if(flowNext.mag < 50 || outward < 0 || (outward == 0 && travel < 0)) {
            onEdge = DmtxFalse;
         }
         else {
            BresLineStep(&line, travel, outward);
            flow = flowNext;
         }
      }

      if(onEdge == DmtxFalse) {
         BresLineStep(&line, 1, 0);
         flow = GetPointFlow(dec, reg->flowBegin.plane, line.loc, dmtxNeighborNone);
         if(flow.mag > 50)
            onEdge = DmtxTrue;
      }

      afterStep = line.loc;
      afterCache = dmtxDecodeGetCache(dec, afterStep.X, afterStep.Y);
      if(afterCache == NULL)
         break;

      /* Определите направление шага, используя чистую магию */
      xStep = afterStep.X - beforeStep.X;
      yStep = afterStep.Y - beforeStep.Y;
      assert(abs(xStep) <= 1 && abs(yStep) <= 1);
      stepDir = dirMap[3 * yStep + xStep + 4];
      assert(stepDir != 8);

      if(streamDir < 0) {
         *beforeCache |= (0x40 | stepDir);
         *afterCache = (((stepDir + 4)%8) << 3);
      }
      else {
         *beforeCache |= (0x40 | (stepDir << 3));
         *afterCache = ((stepDir + 4)%8);
      }

      /* Гарантированно сделал один шаг с начала цикла */
      xDiff = line.loc.X - loc0.X;
      yDiff = line.loc.Y - loc0.Y;
      distSq = (xDiff * xDiff) + (yDiff * yDiff);

      beforeStep = line.loc;
      beforeCache = afterCache;
      steps++;

   } while(distSq < distSqMax);

   return steps;
}

/**
 *
 *
 */
static int
TrailClear(DmtxDecode *dec, DmtxRegion *reg, int clearMask)
{
   int clears;
   DmtxFollow follow;

   assert((clearMask | 0xff) == 0xff);

   /* Clear "visited" bit from trail */
   clears = 0;
   follow = FollowSeek(dec, reg, 0);
   while(abs(follow.step) <= reg->stepsTotal) {
      assert((int)(*follow.ptr & clearMask) != 0x00);
      *follow.ptr &= (clearMask ^ 0xff);
      follow = FollowStep(dec, reg, follow, +1);
      clears++;
   }

   return clears;
}

/**
 * Метод нахождения самой жирной линии
 *
 */
static DmtxBestLine
FindBestSolidLine(DmtxDecode *dec, DmtxRegion *reg, int step0, int step1, int streamDir, int houghAvoid)
{
   int hough[3][DMTX_HOUGH_RES] = { { 0 } };
   int houghMin, houghMax;
   char houghTest[DMTX_HOUGH_RES];
   int i;
   int step;
   int sign;
   int tripSteps;
   int angleBest;
   int hOffset, hOffsetBest;
   int xDiff, yDiff;
   int dH;
   DmtxRay2 rH;
   DmtxFollow follow;
   DmtxBestLine line;
   DmtxPixelLoc rHp;

   memset(&line, 0x00, sizeof(DmtxBestLine));
   memset(&rH, 0x00, sizeof(DmtxRay2));
   angleBest = 0;
   hOffset = hOffsetBest = 0;

   sign = 0;

   /* Всегда следуйте по пути, отходящей от начала тропы */
   if(step0 != 0) {
      if(step0 > 0) {
         sign = +1;
         tripSteps = (step1 - step0 + reg->stepsTotal) % reg->stepsTotal;
      }
      else {
         sign = -1;
         tripSteps = (step0 - step1 + reg->stepsTotal) % reg->stepsTotal;
      }
      if(tripSteps == 0)
         tripSteps = reg->stepsTotal;
   }
   else if(step1 != 0) {
      sign = (step1 > 0) ? +1 : -1;
      tripSteps = abs(step1);
   }
   else if(step1 == 0) {
      sign = +1;
      tripSteps = reg->stepsTotal;
   }
   assert(sign == streamDir);

   follow = FollowSeek(dec, reg, step0);
   rHp = follow.loc;

   line.stepBeg = line.stepPos = line.stepNeg = step0;
   line.locBeg = follow.loc;
   line.locPos = follow.loc;
   line.locNeg = follow.loc;

   /* Заранее определите, какие углы следует тестировать */
   for(i = 0; i < DMTX_HOUGH_RES; i++) {
      if(houghAvoid == DmtxUndefined) {
         houghTest[i] = 1;
      }
      else {
         houghMin = (houghAvoid + DMTX_HOUGH_RES/6) % DMTX_HOUGH_RES;
         houghMax = (houghAvoid - DMTX_HOUGH_RES/6 + DMTX_HOUGH_RES) % DMTX_HOUGH_RES;
         if(houghMin > houghMax)
            houghTest[i] = (i > houghMin || i < houghMax) ? 1 : 0;
         else
            houghTest[i] = (i > houghMin && i < houghMax) ? 1 : 0;
      }
   }

   /* Проверьте каждый угол на наличие шагов по траектории */
   for(step = 0; step < tripSteps; step++) {

      xDiff = follow.loc.X - rHp.X;
      yDiff = follow.loc.Y - rHp.Y;

      /* Накопитель прироста массы */
      for(i = 0; i < DMTX_HOUGH_RES; i++) {

         if((int)houghTest[i] == 0)
            continue;

         dH = (rHvX[i] * yDiff) - (rHvY[i] * xDiff);
         if(dH >= -384 && dH <= 384) {

            if(dH > 128)
               hOffset = 2;
            else if(dH >= -128)
               hOffset = 1;
            else
               hOffset = 0;

            hough[hOffset][i]++;

            /* Новый ракурс takes over lead */
            if(hough[hOffset][i] > hough[hOffsetBest][angleBest]) {
               angleBest = i;
               hOffsetBest = hOffset;
            }
         }
      }

/*    CALLBACK_POINT_PLOT(follow.loc, (sign > 1) ? 4 : 3, 1, 2); */

      follow = FollowStep(dec, reg, follow, sign);
   }

   line.angle = angleBest;
   line.hOffset = hOffsetBest;
   line.mag = hough[hOffsetBest][angleBest];

   return line;
}

/**
 * Метод нахождения самой жирной линии
 *
 */
static DmtxBestLine
FindBestSolidLine2(DmtxDecode *dec, DmtxPixelLoc loc0, int tripSteps, int sign, int houghAvoid)
{
   int hough[3][DMTX_HOUGH_RES] = { { 0 } };
   int houghMin, houghMax;
   char houghTest[DMTX_HOUGH_RES];
   int i;
   int step;
   int angleBest;
   int hOffset, hOffsetBest;
   int xDiff, yDiff;
   int dH;
   DmtxRay2 rH;
   DmtxBestLine line;
   DmtxPixelLoc rHp;
   DmtxFollow follow;

   memset(&line, 0x00, sizeof(DmtxBestLine));
   memset(&rH, 0x00, sizeof(DmtxRay2));
   angleBest = 0;
   hOffset = hOffsetBest = 0;

   follow = FollowSeekLoc(dec, loc0);
   rHp = line.locBeg = line.locPos = line.locNeg = follow.loc;
   line.stepBeg = line.stepPos = line.stepNeg = 0;

   /* Predetermine which angles to test */
   for(i = 0; i < DMTX_HOUGH_RES; i++) {
      if(houghAvoid == DmtxUndefined) {
         houghTest[i] = 1;
      }
      else {
         houghMin = (houghAvoid + DMTX_HOUGH_RES/6) % DMTX_HOUGH_RES;
         houghMax = (houghAvoid - DMTX_HOUGH_RES/6 + DMTX_HOUGH_RES) % DMTX_HOUGH_RES;
         if(houghMin > houghMax)
            houghTest[i] = (i > houghMin || i < houghMax) ? 1 : 0;
         else
            houghTest[i] = (i > houghMin && i < houghMax) ? 1 : 0;
      }
   }

   /* Проверьте каждый угол на наличие шагов по траектории */
   for(step = 0; step < tripSteps; step++) {

      xDiff = follow.loc.X - rHp.X;
      yDiff = follow.loc.Y - rHp.Y;

      /* Increment Hough accumulator */
      for(i = 0; i < DMTX_HOUGH_RES; i++) {

         if((int)houghTest[i] == 0)
            continue;

         dH = (rHvX[i] * yDiff) - (rHvY[i] * xDiff);
         if(dH >= -384 && dH <= 384) {
            if(dH > 128)
               hOffset = 2;
            else if(dH >= -128)
               hOffset = 1;
            else
               hOffset = 0;

            hough[hOffset][i]++;

            /* Новый ракурс takes over lead */
            if(hough[hOffset][i] > hough[hOffsetBest][angleBest]) {
               angleBest = i;
               hOffsetBest = hOffset;
            }
         }
      }

/*    CALLBACK_POINT_PLOT(follow.loc, (sign > 1) ? 4 : 3, 1, 2); */

      follow = FollowStep2(dec, follow, sign);
   }

   line.angle = angleBest;
   line.hOffset = hOffsetBest;
   line.mag = hough[hOffsetBest][angleBest];

   return line;
}

/**
 *
 *
 */
static DmtxPassFail
FindTravelLimits(DmtxDecode *dec, DmtxRegion *reg, DmtxBestLine *line)
{
   int i;
   int distSq, distSqMax;
   int xDiff, yDiff;
   int posRunning, negRunning;
   int posTravel, negTravel;
   int posWander, posWanderMin, posWanderMax, posWanderMinLock, posWanderMaxLock;
   int negWander, negWanderMin, negWanderMax, negWanderMinLock, negWanderMaxLock;
   int cosAngle, sinAngle;
   DmtxFollow followPos, followNeg;
   DmtxPixelLoc loc0, posMax, negMax;

   /* line->stepBeg - уже известно, что он находится на лучшей линии роста */
   followPos = followNeg = FollowSeek(dec, reg, line->stepBeg);
   loc0 = followPos.loc;

   cosAngle = rHvX[line->angle];
   sinAngle = rHvY[line->angle];

   distSqMax = 0;
   posMax = negMax = followPos.loc;

   posTravel = negTravel = 0;
   posWander = posWanderMin = posWanderMax = posWanderMinLock = posWanderMaxLock = 0;
   negWander = negWanderMin = negWanderMax = negWanderMinLock = negWanderMaxLock = 0;

   for(i = 0; i < reg->stepsTotal/2; i++) {

      posRunning = (int)(i < 10 || abs(posWander) < abs(posTravel));
      negRunning = (int)(i < 10 || abs(negWander) < abs(negTravel));

      if(posRunning != 0) {
         xDiff = followPos.loc.X - loc0.X;
         yDiff = followPos.loc.Y - loc0.Y;
         posTravel = (cosAngle * xDiff) + (sinAngle * yDiff);
         posWander = (cosAngle * yDiff) - (sinAngle * xDiff);

         if(posWander >= -3*256 && posWander <= 3*256) {
            distSq = DistanceSquared(followPos.loc, negMax);
            if(distSq > distSqMax) {
               posMax = followPos.loc;
               distSqMax = distSq;
               line->stepPos = followPos.step;
               line->locPos = followPos.loc;
               posWanderMinLock = posWanderMin;
               posWanderMaxLock = posWanderMax;
            }
         }
         else {
            posWanderMin = min(posWanderMin, posWander);
            posWanderMax = max(posWanderMax, posWander);
         }
      }
      else if(!negRunning) {
         break;
      }

      if(negRunning != 0) {
         xDiff = followNeg.loc.X - loc0.X;
         yDiff = followNeg.loc.Y - loc0.Y;
         negTravel = (cosAngle * xDiff) + (sinAngle * yDiff);
         negWander = (cosAngle * yDiff) - (sinAngle * xDiff);

         if(negWander >= -3*256 && negWander < 3*256) {
            distSq = DistanceSquared(followNeg.loc, posMax);
            if(distSq > distSqMax) {
               negMax = followNeg.loc;
               distSqMax = distSq;
               line->stepNeg = followNeg.step;
               line->locNeg = followNeg.loc;
               negWanderMinLock = negWanderMin;
               negWanderMaxLock = negWanderMax;
            }
         }
         else {
            negWanderMin = min(negWanderMin, negWander);
            negWanderMax = max(negWanderMax, negWander);
         }
      }
      else if(!posRunning) {
         break;
      }

/*  CALLBACK_POINT_PLOT(followPos.loc, 2, 1, 2);
    CALLBACK_POINT_PLOT(followNeg.loc, 4, 1, 2); */

      followPos = FollowStep(dec, reg, followPos, +1);
      followNeg = FollowStep(dec, reg, followNeg, -1);
   }
   line->devn = max(posWanderMaxLock - posWanderMinLock, negWanderMaxLock - negWanderMinLock)/256;
   line->distSq = distSqMax;

/* CALLBACK_POINT_PLOT(posMax, 2, 1, 1);
   CALLBACK_POINT_PLOT(negMax, 2, 1, 1); */

   return DmtxPass;
}

/**
 *
 *
 */
static DmtxPassFail
MatrixRegionAlignCalibEdge(DmtxDecode *dec, DmtxRegion *reg, int edgeLoc)
{
   int streamDir;
   int steps;
   int avoidAngle;
   int symbolShape;
   DmtxVector2 pTmp;
   DmtxPixelLoc loc0, loc1, locOrigin;
   DmtxBresLine line;
   DmtxFollow follow;
   DmtxBestLine bestLine;

   /* Определить начало координат */
   pTmp.X = 0.0;
   pTmp.Y = 0.0;
   dmtxMatrix3VMultiplyBy(&pTmp, reg->fit2raw);
   locOrigin.X = (int)(pTmp.X + 0.5);
   locOrigin.Y = (int)(pTmp.Y + 0.5);

   if(dec->sizeIdxExpected == DmtxSymbolSquareAuto ||
         (dec->sizeIdxExpected >= DmtxSymbol10x10 &&
         dec->sizeIdxExpected <= DmtxSymbol144x144))
      symbolShape = DmtxSymbolSquareAuto;
   else if(dec->sizeIdxExpected == DmtxSymbolRectAuto ||
         (dec->sizeIdxExpected >= DmtxSymbol8x18 &&
         dec->sizeIdxExpected <= DmtxSymbol16x48))
      symbolShape = DmtxSymbolRectAuto;
   else
      symbolShape = DmtxSymbolShapeAuto;

   /* Определите конечные местоположения тестовой линии */
   if(edgeLoc == DmtxEdgeTop) {
      streamDir = reg->polarity * -1;
      avoidAngle = reg->leftLine.angle;
      follow = FollowSeekLoc(dec, reg->locT);
      pTmp.X = 0.8;
      pTmp.Y = (symbolShape == DmtxSymbolRectAuto) ? 0.2 : 0.6;
   }
   else {
      assert(edgeLoc == DmtxEdgeRight);
      streamDir = reg->polarity;
      avoidAngle = reg->bottomLine.angle;
      follow = FollowSeekLoc(dec, reg->locR);
      pTmp.X = (symbolShape == DmtxSymbolSquareAuto) ? 0.7 : 0.9;
      pTmp.Y = 0.8;
   }

   dmtxMatrix3VMultiplyBy(&pTmp, reg->fit2raw);
   loc1.X = (int)(pTmp.X + 0.5);
   loc1.Y = (int)(pTmp.Y + 0.5);

   loc0 = follow.loc;
   line = BresLineInit(loc0, loc1, locOrigin);
   steps = TrailBlazeGapped(dec, reg, line, streamDir);

   bestLine = FindBestSolidLine2(dec, loc0, steps, streamDir, avoidAngle);
   // if(bestLine.mag < 5) {
   //   ;
   //}

   if(edgeLoc == DmtxEdgeTop) {
      reg->topKnown = 1;
      reg->topAngle = bestLine.angle;
      reg->topLoc = bestLine.locBeg;
   }
   else {
      reg->rightKnown = 1;
      reg->rightAngle = bestLine.angle;
      reg->rightLoc = bestLine.locBeg;
   }

   return DmtxPass;
}

/**
 *
 *
 */
static DmtxBresLine
BresLineInit(DmtxPixelLoc loc0, DmtxPixelLoc loc1, DmtxPixelLoc locInside)
{
   int cp;
   DmtxBresLine line;
   DmtxPixelLoc *locBeg, *locEnd;

   /* XXX Убедитесь, что loc0 и loc1 являются внутренними */

   /* Значения, которые остаются неизменными после инициализации */
   line.loc0 = loc0;
   line.loc1 = loc1;
   line.xStep = (loc0.X < loc1.X) ? +1 : -1;
   line.yStep = (loc0.Y < loc1.Y) ? +1 : -1;
   line.xDelta = abs(loc1.X - loc0.X);
   line.yDelta = abs(loc1.Y - loc0.Y);
   line.steep = (int)(line.yDelta > line.xDelta);

   /* Возьмите перекрёстный(поперечный) продукт, чтобы определить шаг наружу */
   if(line.steep != 0) {
      /* Направьте первый вектор вверх, чтобы получить правильный знак */
      if(loc0.Y < loc1.Y) {
         locBeg = &loc0;
         locEnd = &loc1;
      }
      else {
         locBeg = &loc1;
         locEnd = &loc0;
      }
      cp = (((locEnd->X - locBeg->X) * (locInside.Y - locEnd->Y)) -
            ((locEnd->Y - locBeg->Y) * (locInside.X - locEnd->X)));

      line.xOut = (cp > 0) ? +1 : -1;
      line.yOut = 0;
   }
   else {
      /* Укажите первый вектор влево, чтобы получить правильный знак */
      if(loc0.X > loc1.X) {
         locBeg = &loc0;
         locEnd = &loc1;
      }
      else {
         locBeg = &loc1;
         locEnd = &loc0;
      }
      cp = (((locEnd->X - locBeg->X) * (locInside.Y - locEnd->Y)) -
            ((locEnd->Y - locBeg->Y) * (locInside.X - locEnd->X)));

      line.xOut = 0;
      line.yOut = (cp > 0) ? +1 : -1;
   }

   /* Значения, которые изменяются при переходе по строке */
   line.loc = loc0;
   line.travel = 0;
   line.outward = 0;
   line.error = (line.steep) ? line.yDelta/2 : line.xDelta/2;

/* CALLBACK_POINT_PLOT(loc0, 3, 1, 1);
   CALLBACK_POINT_PLOT(loc1, 3, 1, 1); */

   return line;
}

/**
 *
 *
 */
static DmtxPassFail
BresLineGetStep(DmtxBresLine line, DmtxPixelLoc target, int *travel, int *outward)
{
   /* Определите необходимый шаг вдоль линии Брезенхема и в сторону от нее */
   if(line.steep != 0) {
      *travel = (line.yStep > 0) ? target.Y - line.loc.Y : line.loc.Y - target.Y;
      BresLineStep(&line, *travel, 0);
      *outward = (line.xOut > 0) ? target.X - line.loc.X : line.loc.X - target.X;
      assert(line.yOut == 0);
   }
   else {
      *travel = (line.xStep > 0) ? target.X - line.loc.X : line.loc.X - target.X;
      BresLineStep(&line, *travel, 0);
      *outward = (line.yOut > 0) ? target.Y - line.loc.Y : line.loc.Y - target.Y;
      assert(line.xOut == 0);
   }

   return DmtxPass;
}

/**
 *
 *
 */
static DmtxPassFail
BresLineStep(DmtxBresLine *line, int travel, int outward)
{
   int i;
   DmtxBresLine lineNew;

   lineNew = *line;

   assert(abs(travel) < 2);
   assert(abs(outward) >= 0);

   /* Выполните шаг вперед */
   if(travel > 0) {
      lineNew.travel++;
      if(lineNew.steep != 0) {
         lineNew.loc.Y += lineNew.yStep;
         lineNew.error -= lineNew.xDelta;
         if(lineNew.error < 0) {
            lineNew.loc.X += lineNew.xStep;
            lineNew.error += lineNew.yDelta;
         }
      }
      else {
         lineNew.loc.X += lineNew.xStep;
         lineNew.error -= lineNew.yDelta;
         if(lineNew.error < 0) {
            lineNew.loc.Y += lineNew.yStep;
            lineNew.error += lineNew.xDelta;
         }
      }
   }
   else if(travel < 0) {
      lineNew.travel--;
      if(lineNew.steep != 0) {
         lineNew.loc.Y -= lineNew.yStep;
         lineNew.error += lineNew.xDelta;
         if(lineNew.error >= lineNew.yDelta) {
            lineNew.loc.X -= lineNew.xStep;
            lineNew.error -= lineNew.yDelta;
         }
      }
      else {
         lineNew.loc.X -= lineNew.xStep;
         lineNew.error += lineNew.yDelta;
         if(lineNew.error >= lineNew.xDelta) {
            lineNew.loc.Y -= lineNew.yStep;
            lineNew.error -= lineNew.xDelta;
         }
      }
   }

   for(i = 0; i < outward; i++) {
      /* Outward steps */
      lineNew.outward++;
      lineNew.loc.X += lineNew.xOut;
      lineNew.loc.Y += lineNew.yOut;
   }

   *line = lineNew;

   return DmtxPass;
}

/**
 *
 *
 */
#ifdef NOTDEFINED
static void
WriteDiagnosticImage(DmtxDecode *dec, DmtxRegion *reg, char *imagePath)
{
   int row, col;
   int width, height;
   unsigned char *cache;
   int rgb[3];
   FILE *fp;
   DmtxVector2 p;
   DmtxImage *img;

   assert(reg != NULL);

   fp = fopen(imagePath, "wb");
   if(fp == NULL) {
      exit(3);
   }

   width = dmtxDecodeGetProp(dec, DmtxPropWidth);
   height = dmtxDecodeGetProp(dec->image, DmtxPropHeight);

   img = dmtxImageCreate(NULL, width, height, DmtxPack24bppRGB);

   /* Заполнить изображение */
   for(row = 0; row < height; row++) {
      for(col = 0; col < width; col++) {

         cache = dmtxDecodeGetCache(dec, col, row);
         if(cache == NULL) {
            rgb[0] = 0;
            rgb[1] = 0;
            rgb[2] = 128;
         }
         else {
            dmtxDecodeGetPixelValue(dec, col, row, 0, &rgb[0]);
            dmtxDecodeGetPixelValue(dec, col, row, 1, &rgb[1]);
            dmtxDecodeGetPixelValue(dec, col, row, 2, &rgb[2]);

            p.X = col;
            p.Y = row;
            dmtxMatrix3VMultiplyBy(&p, reg->raw2fit);

            if(p.X < 0.0 || p.X > 1.0 || p.Y < 0.0 || p.Y > 1.0) {
               rgb[0] = 0;
               rgb[1] = 0;
               rgb[2] = 128;
            }
            else if(p.X + p.Y > 1.0) {
               rgb[0] += (0.4 * (255 - rgb[0]));
               rgb[1] += (0.4 * (255 - rgb[1]));
               rgb[2] += (0.4 * (255 - rgb[2]));
            }
         }

         dmtxImageSetRgb(img, col, row, rgb);
      }
   }

   /* Напишите дополнительные маркеры */
   rgb[0] = 255;
   rgb[1] = 0;
   rgb[2] = 0;
   dmtxImageSetRgb(img, reg->topLoc.X, reg->topLoc.Y, rgb);
   dmtxImageSetRgb(img, reg->rightLoc.X, reg->rightLoc.Y, rgb);

   /* Запись изображения в PNM-файл */
   fprintf(fp, "P6\n%d %d\n255\n", width, height);
   for(row = height - 1; row >= 0; row--) {
      for(col = 0; col < width; col++) {
         dmtxImageGetRgb(img, col, row, rgb);
         fwrite(rgb, sizeof(char), 3, fp);
      }
   }

   dmtxImageDestroy(&img);

   fclose(fp);
}
#endif
