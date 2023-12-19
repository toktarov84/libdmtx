/**
 * libdmtx - Data Matrix Encoding/Decoding Library
 * Copyright 2008, 2009 Mike Laughton. All rights reserved.
 * Copyright 2012-2016 Vadim A. Misbakh-Soloviov. All rights reserved.
 *
 * See LICENSE file in the main project directory for full
 * terms of use and distribution.
 *
 * Contact:
 * Vadim A. Misbakh-Soloviov <dmtx@mva.name>
 * Mike Laughton <mike@dragonflylogic.com>
 *
 * \file dmtxscangrid.c
 * \brief Отслеживание сетки сканирования
 */

/**
 * \brief  Инициализировать шаблон сетки сканирования
 * \param  dec
 * \return Инициализированная сетка
 */
static DmtxScanGrid
InitScanGrid(DmtxDecode *dec)
{
   int scale, smallestFeature;
   int xExtent, yExtent, maxExtent;
   int extent;
   DmtxScanGrid grid;

   memset(&grid, 0x00, sizeof(DmtxScanGrid));

   scale = dmtxDecodeGetProp(dec, DmtxPropScale);
   smallestFeature = dmtxDecodeGetProp(dec, DmtxPropScanGap) / scale;

   grid.xMin = dmtxDecodeGetProp(dec, DmtxPropXmin);
   grid.xMax = dmtxDecodeGetProp(dec, DmtxPropXmax);
   grid.yMin = dmtxDecodeGetProp(dec, DmtxPropYmin);
   grid.yMax = dmtxDecodeGetProp(dec, DmtxPropYmax);

   /* Значения, которые устанавливаются один раз */
   xExtent = grid.xMax - grid.xMin;
   yExtent = grid.yMax - grid.yMin;
   maxExtent = (xExtent > yExtent) ? xExtent : yExtent;

   assert(maxExtent > 1);

   for(extent = 1; extent < maxExtent; extent = ((extent + 1) * 2) - 1)
      if(extent <= smallestFeature)
         grid.minExtent = extent;

   grid.maxExtent = extent;

   grid.xOffset = (grid.xMin + grid.xMax - grid.maxExtent) / 2;
   grid.yOffset = (grid.yMin + grid.yMax - grid.maxExtent) / 2;

   /* Значения, которые сбрасываются для каждого уровня */
   grid.total = 1;
   grid.extent = grid.maxExtent;

   SetDerivedFields(&grid);

   return grid;
}

/**
 * \brief  Верните следующее подходящее местоположение (которое может быть текущим местоположением),
 *         и продвигайтесь по сетке на одну позицию дальше этого. Если ничего хорошего
 *         местоположения остаются, затем возвращается DmtxRangeEnd.
 * \param  grid
 * \return void
 */
static int
PopGridLocation(DmtxScanGrid *grid, DmtxPixelLoc *locPtr)
{
   int locStatus;

   do {
      locStatus = GetGridCoordinates(grid, locPtr);

      /* Всегда оставляйте сетку указывающей на следующее доступное местоположение */
      grid->pixelCount++;

   } while(locStatus == DmtxRangeBad);

   return locStatus;
}

/**
 * \brief  Извлеките текущее положение сетки в пиксельных координатах и верните
 *         является ли местоположение хорошим, плохим или конечным
 * \param  grid
 * \return Местоположение пикселя
 */
static int
GetGridCoordinates(DmtxScanGrid *grid, DmtxPixelLoc *locPtr)
{
   int count, half, quarter;
   DmtxPixelLoc loc;

   /* Первоначально количество пикселей может выходить за допустимые пределы. Обновить сетку
    * состояние перед проверкой координат */

   /* Переход к следующему перекрестному узору по горизонтали, если текущий столбец завершен */
   if(grid->pixelCount >= grid->pixelTotal) {
      grid->pixelCount = 0;
      grid->xCenter += grid->jumpSize;
   }

   /* Переход к следующему перекрестному узору по вертикали, если текущая строка завершена */
   if(grid->xCenter > grid->maxExtent) {
      grid->xCenter = grid->startPos;
      grid->yCenter += grid->jumpSize;
   }

   /* Увеличивайте уровень, когда вертикальный шаг заходит слишком далеко */
   if(grid->yCenter > grid->maxExtent) {
      grid->total *= 4;
      grid->extent /= 2;
      SetDerivedFields(grid);
   }

   if(grid->extent == 0 || grid->extent < grid->minExtent) {
      locPtr->X = locPtr->Y = -1;
      return DmtxRangeEnd;
   }

   count = grid->pixelCount;

   assert(count < grid->pixelTotal);

   if(count == grid->pixelTotal - 1) {
      /* center pixel */
      loc.X = grid->xCenter;
      loc.Y = grid->yCenter;
   }
   else {
      half = grid->pixelTotal / 2;
      quarter = half / 2;

      /* горизонтальная позиция */
      if(count < half) {
         loc.X = grid->xCenter + ((count < quarter) ? (count - quarter) : (half - count));
         loc.Y = grid->yCenter;
      }
      /* вертикальная позиция */
      else {
         count -= half;
         loc.X = grid->xCenter;
         loc.Y = grid->yCenter + ((count < quarter) ? (count - quarter) : (half - count));
      }
   }

   loc.X += grid->xOffset;
   loc.Y += grid->yOffset;

   *locPtr = loc;

   if(loc.X < grid->xMin || loc.X > grid->xMax ||
         loc.Y < grid->yMin || loc.Y > grid->yMax)
      return DmtxRangeBad;

   return DmtxRangeGood;
}

/**
 * \brief  Обновлять производные поля на основе текущего состояния
 * \param  grid
 * \return void
 */
static void
SetDerivedFields(DmtxScanGrid *grid)
{
   grid->jumpSize = grid->extent + 1;
   grid->pixelTotal = 2 * grid->extent - 1;
   grid->startPos = grid->extent / 2;
   grid->pixelCount = 0;
   grid->xCenter = grid->yCenter = grid->startPos;
}
